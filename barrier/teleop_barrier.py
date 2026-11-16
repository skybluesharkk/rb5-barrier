#!/usr/bin/env python3
"""Quest 손 텔레옵 + 박스 바리어 + 로봇 손끝 기록.

    conda activate dtrecon-capture        # rbpodo, pyzmq, opencv, matplotlib 있는 환경
    cd ~/rb5-barrier
    python -m barrier.teleop_barrier --mode hard --tag slow_x            # 실제 로봇 (Wi-Fi), Quest 필요
    python -m barrier.teleop_barrier --mode cbf --alpha 5 --tag cbf_fast
    python -m barrier.teleop_barrier --sim ...                           # 컨트롤러 시뮬레이션 모드 (팔 안 움직임, tcp_ref만)
    python -m barrier.teleop_barrier --fake-robot --fake-quest 0.10 --mode hard --tag test   # 하드웨어 전혀 없이

핀치: Middle pinch(파란) → 추적 시작 (그 순간 손·로봇 위치를 기준점으로), Index pinch(초록) → 정지.
Ctrl+C 로 종료. 결과는 logs/<날짜시각>_<tag>/ 에 cmd.csv, state.csv, meta.json → analyze.py 로 그래프.

제어 (틱마다, dt 측정값):
    p_des = p_init + scale · R_quest→robot · (wrist − wrist_init)
    v_des = clip((p_des − p_cmd)/dt, v_max)
    v_cmd = 실험박스.project(p_cmd, v_des, mode, α) → 안전박스.project(hard)
    p_cmd += v_cmd·dt  → move_servo_l(p_cmd, 자세는 시작 때 값 고정)
hard 모드에서 손이 벽 밖에 있으면 p_cmd 는 벽에 붙어 있고, 손이 박스 안으로 돌아와야 다시 움직인다 ("벽을 계속 미는 상황").
"""
import argparse
import csv
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

from .box import FACES, NAMES, Cylinder, parse_box
from .quest import R_QUEST_TO_ROBOT, FakeQuest, QuestReceiver, hand_matrix, yaw_to_align
from .robot import EMS_NAMES, FakeRobot, RB5Servo, StateLogger

HERE = Path(__file__).parent
CMD_FIELDS = (["t", "dt", "tracking", "mode", "wrist_qx", "wrist_qy", "wrist_qz"]
              + [f"hand_{a}" for a in "xyz"]                       # 손 위치를 로봇 좌표로 (p_init + scale·R·Δwrist = p_des 와 같음, 추적 전엔 NaN)
              + [f"p_des_{a}" for a in "xyz"] + [f"v_des_{a}" for a in "xyz"] + [f"v_cmd_{a}" for a in "xyz"]
              + [f"p_cmd_{a}" for a in "xyz"] + [f"d_{f}" for f in NAMES] + ["active", "safety_active"]
              + [f"tcp_meas_{a}" for a in "xyz"] + [f"tcp_ref_{a}" for a in "xyz"] + ["state_age"])   # 로봇 (가장 최근 상태, m)


def parse_args():
    cfg = yaml.safe_load((HERE / "barrier.yaml").read_text())
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default=cfg["robot_ip"])
    ap.add_argument("--host", default=cfg["host"], help="ZMQ bind 주소")
    ap.add_argument("--hz", type=float, default=cfg["hz"])
    ap.add_argument("--speed", type=float, default=cfg["speed_bar"], help="speed_bar 0~1")
    ap.add_argument("--scale", type=float, default=cfg["scale"])
    ap.add_argument("--v-max", type=float, default=cfg["v_max"])
    ap.add_argument("--t1", type=float, default=cfg["servo"]["t1"])
    ap.add_argument("--t2", type=float, default=cfg["servo"]["t2"])
    ap.add_argument("--mode", choices=["none", "hard", "cbf"], default=cfg["barrier"]["mode"])
    ap.add_argument("--alpha", type=float, default=cfg["barrier"]["alpha"], help="cbf: v·n ≥ -alpha·d")
    ap.add_argument("--yaw", type=float, default=cfg["hand_frame"]["yaw_deg"], help="손 좌표 yaw 보정 (deg, 로봇 z 축)")
    ap.add_argument("--calib-dist", type=float, default=cfg["hand_frame"]["calib_dist"],
                    help="추적 시작 후 손을 앞으로 이만큼(m) 뻗으면 그 방향을 user_forward 로 맞춰 yaw 자동 보정. 0 = 끔")
    ap.add_argument("--box", default=None, help="xmin,xmax,ymin,ymax,zmin,zmax (m). 기본 barrier.yaml")
    ap.add_argument("--r-min", type=float, default=cfg.get("r_min", 0.25), help="base 기둥 원기둥 반지름 (m). 0 = 없음")
    ap.add_argument("--sim", action="store_true", help="컨트롤러 시뮬레이션 모드 (팔 안 움직임)")
    ap.add_argument("--fake-robot", action="store_true", help="rbpodo 없이 가짜 로봇")
    ap.add_argument("--fake-quest", type=float, default=None, metavar="SPEED",
                    help="헤드셋 없이: 손이 SPEED m/s 로 --fake-axis 방향 벽을 향해 간다")
    ap.add_argument("--fake-axis", default="x", help="fake-quest 이동 축: x, -x, y, -y, z, -z")
    ap.add_argument("--fake-travel", type=float, default=0.90, help="fake-quest 이동 거리 (m). 벽 너머까지 (박스 ±0.75 기준)")
    ap.add_argument("--no-panel", action="store_true", help="Quest 패널 송신 안 함")
    ap.add_argument("--cam", type=int, default=-1, metavar="N", help="웹캠 번호(/dev/videoN)를 Quest 카메라 패널(10505)로 송신. 로봇을 비추게 둘 것. -1 = 검은 화면")
    ap.add_argument("--no-viewer", action="store_true", help="브라우저 3D 뷰어 안 띄움")
    ap.add_argument("--viewer-port", type=int, default=8766)
    ap.add_argument("--no-browser", action="store_true", help="뷰어를 브라우저로 자동으로 열지 않음")
    ap.add_argument("--mqtt", default=None, metavar="BROKER[:PORT]", help="선택: 관절각·바리어 상태를 MQTT 로도 발행 (Jane 언리얼용)")
    ap.add_argument("--tag", default="run")
    ap.add_argument("--logdir", default=str(HERE / "logs"))
    ap.add_argument("--duration", type=float, default=0.0, help="초. 0 = Ctrl+C 까지")
    a = ap.parse_args()
    a.box = parse_box(a.box or cfg["box"])
    a.safety_box = parse_box(cfg["safety_box"])
    a.cyl = Cylinder(a.r_min) if a.r_min > 0 else None
    a.servo = dict(t1=a.t1, t2=a.t2, gain=cfg["servo"]["gain"], alpha=cfg["servo"]["alpha"])
    a.user_forward = np.array(cfg["hand_frame"]["user_forward"], float)
    a.quest_cfg = {**dict(jump_guard_m=0.0, ignore_zero=True, stall_restart=False, timeout_restart=False), **cfg.get("quest", {})}
    if a.fake_quest is not None:        # 가짜 손은 R_QUEST_TO_ROBOT 기준으로 궤적을 만든다 → 보정 없음
        a.yaw, a.calib_dist = 0.0, 0.0
    return a


def main():
    a = parse_args()
    run_dir = Path(a.logdir) / f"{datetime.now():%Y%m%d_%H%M%S}_{a.tag}"
    run_dir.mkdir(parents=True)
    t0 = time.time()

    # ── 로봇 ──
    if a.fake_robot:
        robot = FakeRobot(**a.servo).connect()
        print("가짜 로봇")
    else:
        print(f"로봇 연결 {a.robot_ip} ({'시뮬레이션' if a.sim else '실제'} 모드, speed_bar {a.speed})")
        robot = RB5Servo(a.robot_ip, speed_bar=a.speed, simulation=a.sim, **a.servo).connect()
    pose0 = robot.tcp_pose6()
    p0 = pose0[:3] / 1000.0
    print(f"TCP {pose0[0]:.1f} {pose0[1]:.1f} {pose0[2]:.1f} mm")
    d0 = np.concatenate([a.box.distances(p0), a.cyl.distances(p0) if a.cyl else [np.inf]])
    if (d0 < 0).any():
        bad = [f"{f}({v*100:.1f}cm)" for f, v in zip(NAMES, d0) if v < 0]
        print(f"오류: 시작 TCP가 실험 박스 밖 — {', '.join(bad)}. 펜던트/대시보드로 박스 안에 넣고 다시 실행")
        robot.close()
        sys.exit(1)
    if not a.fake_robot:
        st0 = robot.read_state()
        if st0 is not None:
            q0 = st0["jnt_ang"]
            print(f"관절 {np.round(q0, 1)}")
            if abs(abs(q0[4]) - 180.0) < 25 or abs(q0[4]) < 25:
                print(f"경고: J5={q0[4]:.1f}° 로 손목 특이점(0°/±180°) 근처에서 시작. 조금만 움직여도 컨트롤러가 멈출 수 있음 — "
                      f"펜던트로 J5 를 -90° 근처(손목 굽힘)로 옮기고 다시 실행하는 것을 권장")
            r_xy = np.hypot(p0[0], p0[1])
            if r_xy > 0.70:
                print(f"경고: 시작 자세의 수평 반경 {r_xy*100:.0f} cm — 팔이 거의 다 뻗어 있음(도달 한계 ≈ 85~90 cm). 안쪽에서 시작 권장")

    # ── Quest ──
    if a.fake_quest is not None:
        quest = FakeQuest(axis=a.fake_axis, speed=a.fake_quest, travel=a.fake_travel, scale=a.scale)
        print(f"가짜 손: {a.fake_axis} 방향 {a.fake_quest} m/s, {a.fake_travel} m")
    else:
        qc = a.quest_cfg
        quest = QuestReceiver(a.host, jump_guard_m=qc["jump_guard_m"], ignore_zero=qc["ignore_zero"],
                              log_path=run_dir / "quest.csv", t0=t0)
        jg = "끔" if qc["jump_guard_m"] <= 0 else f"{qc['jump_guard_m']} m"
        print(f"점프가드 {jg}, (0,0,0) 무시 {qc['ignore_zero']}, 못따라옴 자동재시작 {qc['stall_restart']}, "
              f"패킷끊김 자동재시작 {qc['timeout_restart']}")
        print(f"Quest 수신 대기 (ZMQ {a.host}:8087). Quest 앱에서 이 PC IP로 스트림 시작")

    # ── 패널 ──
    panel = None
    if not a.no_panel:
        from .panel import PanelProxy
        panel = PanelProxy(a.host, cam_index=a.cam)
        panel.push(mode=a.mode, alpha=a.alpha, scale=a.scale, tracking=False)

    viewer = None
    if not a.no_viewer:
        from .viewer import ViewerProxy
        viewer = ViewerProxy(a.viewer_port)
        viewer.push(box=a.box.to_dict(), safety_box=a.safety_box.to_dict(), r_min=a.r_min, mode=a.mode, alpha=a.alpha, scale=a.scale,
                    tracking=False, T_flange_tcp=getattr(robot, "T_flange_tcp", None))
        print(f"3D 뷰어: http://localhost:{a.viewer_port}  (같은 네트워크의 다른 PC는 이 PC IP로)")
        if not a.no_browser:
            import webbrowser
            webbrowser.open(f"http://localhost:{a.viewer_port}")

    mq = None
    if a.mqtt:
        from .mqtt import MqttPublisher
        host, _, port = a.mqtt.partition(":")
        mq = MqttPublisher(host, int(port or 1883))
        print(f"MQTT 발행 {a.mqtt}: topic/robot, topic/barrier")

    meta = dict(tag=a.tag, t0=t0, mode=a.mode, alpha=a.alpha, hz=a.hz, scale=a.scale, v_max=a.v_max, servo=a.servo,
                box=a.box.to_dict(), safety_box=a.safety_box.to_dict(), r_min=a.r_min, robot_ip=a.robot_ip, sim=a.sim,
                fake_robot=a.fake_robot, fake_quest=a.fake_quest, fake_axis=a.fake_axis, start_tcp_mm=pose0.tolist(),
                R_quest_to_robot=R_QUEST_TO_ROBOT.tolist(), yaw_deg=a.yaw, user_forward=a.user_forward.tolist(),
                calib_dist=a.calib_dist, quest=a.quest_cfg)
    (run_dir / "meta.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
    slog = StateLogger(robot, run_dir / "state.csv", t0).start()
    cmd_f = open(run_dir / "cmd.csv", "w", newline="")
    cmd_w = csv.writer(cmd_f)
    cmd_w.writerow(CMD_FIELDS)

    stop = False

    def on_sig(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    nan3 = [float("nan")] * 3

    def log_tick(dt, tracking, mode, wrist, p_des=None, v_des=None, v_cmd=None, p_cmd=None, info=None, sinfo=None):
        st = slog.latest
        meas = st["tcp_meas"] if st else nan3
        ref = (st["tcp_ref"][:3] / 1000.0) if st else nan3
        age = (time.time() - st["t"]) if st else float("nan")
        f5 = lambda v: [f"{x:.5f}" for x in (v if v is not None else nan3)]
        f4 = lambda v: [f"{x:.4f}" for x in (v if v is not None else nan3)]
        d = info["d"] if info else [float("nan")] * len(NAMES)
        act = "".join("1" if x else "0" for x in info["active"]) if info else "0" * len(NAMES)
        sact = "".join("1" if x else "0" for x in sinfo["active"]) if sinfo else "000000"
        cmd_w.writerow([f"{time.time() - t0:.4f}", f"{dt:.5f}", int(tracking), mode, *f5(wrist), *f5(p_des), *f5(p_des),
                        *f4(v_des), *f4(v_cmd), *f5(p_cmd), *f4(d), act, sact, *f5(meas), *f5(ref), f"{age:.3f}"])

    period = 1.0 / a.hz
    tracking = False
    last_mode = "UNKNOWN"
    wrist_init = p_init = p_cmd = None
    calibrating = False
    R_hand = hand_matrix(a.yaw)
    stall_since = None          # 컨트롤러 참조(tcp_ref)가 p_cmd 를 못 따라오기 시작한 시각
    stall_warned = False
    timeout_warned = False
    last_flags = (0, 0, 0, 0, 0)
    pose_cmd = pose0.copy()
    t_prev = time.perf_counter()
    n_tick = 0
    dts = []
    print(f"박스 {a.box.to_dict()}\n모드 {a.mode}" + (f" α={a.alpha}" if a.mode == "cbf" else "") + f"  {a.hz:.0f} Hz  기록 {run_dir}")
    print("Middle pinch → 추적 시작, Index pinch → 정지, Ctrl+C → 종료")
    if a.calib_dist > 0:
        print(f"추적 시작 직후 손을 로봇 쪽(앞)으로 {a.calib_dist*100:.0f} cm 뻗으면 좌표계 yaw 를 자동 보정한다 (그 전엔 로봇이 안 움직임)\n")
    else:
        print(f"yaw 보정 {a.yaw:+.1f}° 고정\n")
    try:
        while not stop:
            wrist, mode, _ = quest.poll()
            if mode != last_mode and not (mode == "CONT" and quest.age() > 0.5):   # 패킷이 안 오는 동안엔 재시작하지 않음
                if mode == "CONT" and wrist is not None:
                    wrist_init = wrist.copy()
                    pose_cmd = robot.tcp_pose6()
                    p_init = pose_cmd[:3] / 1000.0
                    p_cmd = p_init.copy()
                    tracking = True
                    stall_since = None
                    calibrating = a.calib_dist > 0
                    print(f"\n[추적 시작] 로봇 {p_init*1000}" + ("  → 손을 앞으로 뻗으세요 (yaw 보정)" if calibrating else ""))
                elif mode == "STOP":
                    tracking = False
                    print("\n[정지] Middle pinch 로 재시작")
                last_mode = mode
                if panel:
                    panel.push(tracking=tracking, d=None, active=None)
                if viewer:
                    viewer.push(tracking=tracking)
            now = time.perf_counter()
            if now - t_prev < period:
                time.sleep(0.0005)
                continue
            dt = now - t_prev
            t_prev = now
            n_tick += 1
            dts.append(dt)
            if slog.latest is not None:
                if panel and n_tick % 5 == 0:
                    panel.push(tcp=slog.latest["tcp_ref"][:3])
                if viewer and n_tick % 2 == 0:
                    if a.sim:
                        viewer.push(jnt_ang=slog.latest["jnt_ref"], tcp_meas=slog.latest["tcp_ref"][:3] / 1000.0)
                    else:
                        viewer.push(jnt_ang=slog.latest["jnt_ang"], tcp_meas=slog.latest["tcp_meas"])
                if mq and n_tick % 2 == 0:
                    mq.joints(slog.latest["jnt_ang"])
            if a.duration and time.time() - t0 > a.duration:
                break
            if isinstance(quest, FakeQuest) and quest.done():
                break
            if not tracking or wrist is None:
                log_tick(dt, False, mode, wrist)
                if n_tick % 100 == 0 and not isinstance(quest, FakeQuest):     # 2초마다: Quest 가 붙었는지 보여 준다
                    n = quest.n_packets
                    print(f"\r[대기] Quest 패킷 총 {n}  ({'마지막 수신 %.1fs 전' % quest.age() if n else '아직 없음'})  "
                          f"모드 {mode}  → 왼손 Middle pinch 로 시작   ", end="", flush=True)
                continue
            if quest.age() > 0.5:
                if a.quest_cfg["timeout_restart"]:          # servo_teleop 방식: 중단 → 패킷 재개 시 그 순간 위치로 재기준
                    print(f"\n경고: Quest 패킷 {quest.age():.2f} s 없음 (t={time.time()-t0:.2f}) — 추적 중단 (재개 시 재기준)")
                    tracking = False
                    last_mode = "UNKNOWN"
                    if panel:
                        panel.push(tracking=False)
                    continue
                if not timeout_warned:                      # 꺼져 있으면: 경고 한 번, 마지막 목표 유지, 재개되면 원래 기준점 그대로 이어감
                    timeout_warned = True
                    print(f"\n경고: Quest 패킷 {quest.age():.2f} s 없음 (t={time.time()-t0:.2f}) — 마지막 목표 유지 (처리 없음)")
            else:
                timeout_warned = False

            if calibrating:                       # 손이 "앞"으로 calib_dist 만큼 가면 그 수평 방향을 user_forward 로 맞춘다
                delta = R_QUEST_TO_ROBOT @ (wrist - wrist_init)
                if np.hypot(delta[0], delta[1]) >= a.calib_dist:
                    yaw = yaw_to_align(delta, a.user_forward)
                    R_hand = hand_matrix(yaw)
                    wrist_init = wrist.copy()     # 여기서부터 움직임. 로봇은 아직 p_init 그대로
                    calibrating = False
                    meta["yaw_deg"] = yaw
                    (run_dir / "meta.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
                    print(f"\n[보정 완료] yaw {yaw:+.1f}°  (손 앞 방향 → 로봇 {a.user_forward})")
                else:
                    log_tick(dt, False, mode, wrist)
                    continue
            p_des = p_init + a.scale * (R_hand @ (wrist - wrist_init))
            v_des = (p_des - p_cmd) / dt
            n = np.linalg.norm(v_des)
            if n > a.v_max:
                v_des *= a.v_max / n
            v_cmd, info = a.box.project(p_cmd, v_des, dt, a.mode, a.alpha)
            if a.cyl:
                v_cmd, cinfo = a.cyl.project(p_cmd, v_cmd, dt, a.mode, a.alpha)
            else:
                cinfo = dict(d=np.array([np.inf]), active=np.zeros(1, bool))
            info = dict(d=np.concatenate([info["d"], cinfo["d"]]), active=np.concatenate([info["active"], cinfo["active"]]))
            v_cmd, sinfo = a.safety_box.project(p_cmd, v_cmd, dt, "hard")
            p_cmd = p_cmd + v_cmd * dt
            pose_cmd[:3] = p_cmd * 1000.0
            robot.servo(pose_cmd)

            log_tick(dt, True, mode, wrist, p_des, v_des, v_cmd, p_cmd, info, sinfo)

            # 컨트롤러가 서보를 멈췄는지 감시: 참조 TCP 가 p_cmd 에서 3 cm 이상 떨어진 채 0.5 s 지나면 멈춘 것 (특이점·관절한계·팔 뻗음).
            # 그대로 두면 컨트롤러가 다시 받기 시작할 때 p_cmd 까지 한 번에 튀므로 추적을 끊고 재핀치(= 현재 자세에서 재기준)하게 한다.
            st = slog.latest
            if st is not None and not a.sim and not a.fake_robot:
                flags = tuple(st.get("flags", last_flags))
                if flags != last_flags and any(f > 0 for f in flags):
                    ems, col, scol, sestop, sos = flags
                    print(f"\n경고: 컨트롤러 플래그 ems={ems}{(' ' + EMS_NAMES.get(ems, '')) if ems else ''} collision={col} "
                          f"self_collision={scol} pause={sestop} sos={sos}  관절 {np.round(st['jnt_ang'], 1)}")
                last_flags = flags
                gap = np.linalg.norm(p_cmd - st["tcp_ref"][:3] / 1000.0)
                if gap > 0.03:
                    stall_since = stall_since or now
                    if now - stall_since > 0.5 and not stall_warned:
                        stall_warned = True
                        q = np.round(st["jnt_ang"], 1)
                        print(f"\n경고: 로봇이 명령을 못 따라옴 (p_cmd 와 컨트롤러 참조 {gap*100:.1f} cm 차이, 바리어 아님). "
                              f"관절 {q} — J5 가 ±180° 근처면 손목 특이점, 팔이 쭉 뻗었으면 도달 한계."
                              + ("\n      현재 로봇 자세를 새 기준으로 재시작" if a.quest_cfg["stall_restart"] else ""))
                    if now - stall_since > 0.5 and a.quest_cfg["stall_restart"]:    # quest.stall_restart: false 면 경고만
                        tracking = False
                        last_mode = "UNKNOWN"
                        stall_since = None
                        if panel:
                            panel.push(tracking=False)
                        if viewer:
                            viewer.push(tracking=False)
                        continue
                else:
                    stall_since, stall_warned = None, False
            if viewer and n_tick % 2 == 0:
                viewer.push(p_cmd=p_cmd, p_des=p_des, d=info["d"], active=info["active"])
            if mq and n_tick % 2 == 0:
                mq.barrier(mode=a.mode, alpha=a.alpha, tracking=True, p_cmd=p_cmd, p_des=p_des, d=info["d"], active=info["active"])
            if n_tick % 5 == 0:
                if panel:
                    panel.push(d=info["d"], active=info["active"])
                hit = [f for f, x in zip(NAMES, info["active"]) if x]
                print(f"\r[{a.mode}] p_cmd {p_cmd[0]*100:6.1f} {p_cmd[1]*100:6.1f} {p_cmd[2]*100:6.1f} cm  "
                      f"|v_des| {np.linalg.norm(v_des):.3f} |v_cmd| {np.linalg.norm(v_cmd):.3f} m/s  "
                      f"d_min {info['d'].min()*100:5.1f} cm  {'HIT ' + ','.join(hit) if hit else '    '}   ", end="", flush=True)
    finally:
        print("\n종료 중...")
        slog.stop()
        cmd_f.close()
        quest.close()
        if panel:
            panel.close()
        if viewer:
            viewer.close()
        if mq:
            mq.close()
        robot.close()
        if dts:
            d = np.array(dts)
            meta.update(ticks=len(d), dt_mean=float(d.mean()), dt_max=float(d.max()), dt_p99=float(np.percentile(d, 99)),
                        state_rows=slog.n, state_fail=slog.n_fail)
            (run_dir / "meta.json").write_text(json.dumps(meta, indent=1, ensure_ascii=False))
            print(f"루프 dt 평균 {d.mean()*1000:.1f} ms, 최대 {d.max()*1000:.1f} ms, p99 {np.percentile(d, 99)*1000:.1f} ms; "
                  f"상태 {slog.n}행 (실패 {slog.n_fail})")
        print(f"기록: {run_dir}\n그래프: python -m barrier.analyze {run_dir}")


if __name__ == "__main__":
    main()
