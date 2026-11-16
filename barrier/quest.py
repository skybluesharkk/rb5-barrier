"""Quest(Open-Teach APK) 손 키포인트 수신 + 헤드셋 없는 가짜 손.

servo_teleop.py의 수신·점프가드·핀치 모드 로직을 그대로 떼어 왔다.
  PULL 8087 키포인트 "absolute:x,y,z|x,y,z|..." 24개, 0번 = 손목
  PULL 8095 버튼 (미사용), PULL 8100 모드 "High"=추적 / "Low"=정지
"""
import re
import time

import numpy as np

NUM_KEYPOINTS = 24
WRIST_INDEX = 0

# Quest 좌표 → 로봇 base 좌표 (servo_teleop.py와 동일). 로봇 x = Quest z, 로봇 y = -Quest x, 로봇 z = Quest y
R_QUEST_TO_ROBOT = np.array([[0, 0, 1], [-1, 0, 0], [0, 1, 0]], float)


def rot_z(yaw_deg):
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


def hand_matrix(yaw_deg=0.0):
    """Quest → 로봇 변환에 로봇 z 축 yaw 를 더한 행렬. Quest 추적 원점의 yaw 는 앱마다 달라서 보정이 필요."""
    return rot_z(yaw_deg) @ R_QUEST_TO_ROBOT


def yaw_to_align(delta_robot, user_forward):
    """손을 "앞"으로 뻗은 변위(delta_robot, 로봇 좌표·보정 전)가 user_forward 를 가리키게 하는 yaw(deg).
    둘 다 수평 성분만 쓴다."""
    a = np.arctan2(delta_robot[1], delta_robot[0])
    b = np.arctan2(user_forward[1], user_forward[0])
    return float(np.degrees((b - a + np.pi) % (2 * np.pi) - np.pi))


def parse_keypoints(packet: bytes):
    """"absolute:x,y,z|..." → (prefix, (24,3)). prefix 는 앱 모드: absolute = 팔 모드(중지 핀치, 파랑), relative = 손만(검지, 초록)."""
    text = packet.decode().strip()
    if ":" not in text:
        raise ValueError(f"예상치 못한 패킷: {text[:80]!r}")
    prefix, payload = text.split(":", 1)
    vals = [float(v) for v in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", payload)]
    if len(vals) < NUM_KEYPOINTS * 3:
        raise ValueError(f"키포인트 부족: {len(vals)//3}/{NUM_KEYPOINTS}")
    return prefix.strip().lower(), np.asarray(vals[:NUM_KEYPOINTS * 3]).reshape(NUM_KEYPOINTS, 3)


TRACK_FIELDS = ("tracked", "hand_conf", "thumb_conf", "index_conf", "middle_conf", "ring_conf", "pinky_conf",
                "data_valid", "high_conf", "unity_t")


def parse_tracking_state(packet: bytes):
    """패치된 APK(quest_app_patch/)가 키포인트 뒤 세 번째 ':' 필드로 붙이는 추적 신뢰도. 없으면 None.
    "absolute:<kps>:tracked,handConf,thumb,index,middle,ring,pinky,dataValid,highConf,unityTime"  (conf 는 0.0~1.0 실수) """
    parts = packet.decode(errors="ignore").strip().split(":")
    if len(parts) < 3 or not parts[2].strip():
        return None
    vals = parts[2].strip().split(",")
    if len(vals) < len(TRACK_FIELDS):
        return None
    try:
        out = {k: float(v) for k, v in zip(TRACK_FIELDS, vals)}
        for k in ("tracked", "data_valid", "high_conf"):
            out[k] = int(out[k])                               # 신뢰도 6개는 0.0~1.0 실수, 나머지는 0/1
    except ValueError:
        return None
    return out


class QuestReceiver:
    """poll() 마다 (wrist[m] or None, mode 'CONT'|'STOP'|'UNKNOWN', fresh) 를 돌려준다."""

    def __init__(self, host="0.0.0.0", keypoint_port=8087, button_port=8095, teleop_port=8100,
                 jump_guard_m=0.10, lost_accept_s=1.0, ignore_zero=True, log_path=None, t0=None):
        import zmq
        self._log = None
        self._t0 = t0 if t0 is not None else time.time()
        if log_path:                                   # quest.csv: Quest 가 보낸 패킷 전부 (수신 시각 기준, ~70 Hz)
            import csv
            self._logf = open(log_path, "w", newline="")
            self._log = csv.writer(self._logf)
            self._log.writerow(["t", "prefix", "wrist_qx", "wrist_qy", "wrist_qz", "accepted", *TRACK_FIELDS])   # 신뢰도 열은 패치 APK 일 때만 채워짐
        self.zmq = zmq
        self.ctx = zmq.Context()
        self.key = self._pull(host, keypoint_port)
        self.btn = self._pull(host, button_port)
        self.tel = self._pull(host, teleop_port)
        self.jump_guard_m, self.lost_accept_s = jump_guard_m, lost_accept_s     # jump_guard_m <= 0 → 점프가드 끔
        self.ignore_zero = ignore_zero
        self.wrist = None
        self.wrist_last_valid = None
        self.mode = "UNKNOWN"
        self.last_t = 0.0
        self.lost = False            # 점프 거부 중
        self.lost_since = 0.0
        self.hand_zero = False       # 마지막 패킷이 (0,0,0) = Quest 가 손을 못 보는 중
        self.zero_warned = False
        self.track = None            # 패치 APK 의 추적 신뢰도 (parse_tracking_state), 원본 APK 면 None
        self.n_packets = 0
        self.last_tel = None   # 8100 에서 마지막으로 받은 원문 (진단용)
        self.last_btn = None   # 8095
        self.last_prefix = None

    def _pull(self, host, port):
        s = self.ctx.socket(self.zmq.PULL)
        s.setsockopt(self.zmq.CONFLATE, 1)
        s.setsockopt(self.zmq.RCVTIMEO, 1)
        s.bind(f"tcp://{host}:{port}")
        return s

    def _recv(self, sock):
        try:
            return sock.recv(flags=self.zmq.NOBLOCK)
        except self.zmq.Again:
            return None

    def poll(self):
        now = time.time()
        tel = self._recv(self.tel)
        btn = self._recv(self.btn)
        if btn is not None:
            self.last_btn = btn[:40]
        if tel is not None:
            self.last_tel = tel[:40]
            tok = tel.decode(errors="ignore").strip()
            self.mode = {"Low": "STOP", "High": "CONT"}.get(tok, self.mode)
        pkt = self._recv(self.key)
        fresh = False
        if pkt is not None:
            try:
                prefix, kps = parse_keypoints(pkt)
                w = kps[WRIST_INDEX]
                self.track = parse_tracking_state(pkt)      # 패치 APK 면 dict, 아니면 None
            except ValueError as e:
                print(f"\n파싱 오류: {e}")
                return self.wrist, self.mode, False
            # 모드는 키포인트 패킷의 접두어가 가장 확실하다 (v9 빌드는 8100 포트로 아무것도 안 보냄 — 2026-10-23 확인)
            if prefix == "absolute":
                self.mode = "CONT"
            elif prefix == "relative":
                self.mode = "STOP"
            self.last_prefix = prefix
            self.n_packets += 1
            self.last_t = now
            accepted = True
            # Quest 는 손을 못 보면(카메라 가림) 좌표 (0,0,0) 을 계속 보낸다 (2026-10-27 로그). 손 위치로 쓰지 않고 마지막 유효값을 유지.
            self.hand_zero = bool(np.linalg.norm(w) < 1e-6)
            if self.hand_zero and self.ignore_zero:
                accepted = False
                if not self.zero_warned:
                    self.zero_warned = True
                    print("\n경고: Quest 가 (0,0,0) 전송 — 손이 카메라에 안 보임. 마지막 손 위치 유지")
            else:
                self.zero_warned = False
                if self.wrist_last_valid is not None and self.jump_guard_m > 0:
                    # servo_teleop 과 동일: 10 cm 이상 점프는 거부하다가 1 s 넘게 계속되면 새 좌표계로 수용
                    jump = np.linalg.norm(w - self.wrist_last_valid)
                    if jump > self.jump_guard_m:
                        accepted = False
                        if not self.lost:
                            self.lost, self.lost_since = True, now
                            print(f"\n경고: 트래킹 손실 의심 ({jump*100:.0f}cm 점프) — 복구 대기")
                        elif now - self.lost_since > self.lost_accept_s:
                            self.wrist_last_valid = w.copy()
                            self.lost = False
                            print("\n[수용] 새 좌표계로 간주")
                    else:
                        self.lost = False
            if self._log:
                tr = self.track or {}
                self._log.writerow([f"{now - self._t0:.4f}", prefix, f"{w[0]:.5f}", f"{w[1]:.5f}", f"{w[2]:.5f}", int(accepted),
                                    *[tr.get(k, "") for k in TRACK_FIELDS]])
            if not accepted:
                return self.wrist, self.mode, False
            self.wrist, self.wrist_last_valid = w, w.copy()
            fresh = True
        return self.wrist, self.mode, fresh

    def age(self):
        return time.time() - self.last_t

    def close(self):
        for s in (self.key, self.btn, self.tel):
            s.close(0)
        self.ctx.term()
        if self._log:
            self._logf.close()


class FakeQuest:
    """헤드셋 없이 벽 접근 시나리오를 만든다 (로봇 좌표로 정의하고 Quest 좌표로 되돌려 보냄).

    t<1s    : 정지 (STOP) → 1s에 CONT 로 전환(추적 시작, 기준점 래치)
    1~2s    : 정지
    이후     : axis 방향으로 speed [m/s]로 travel [m]만큼 이동 (벽 너머까지) → hold 초 정지 → 같은 속도로 복귀
    손 이동량은 scale 로 나눠서 보낸다 (로봇이 travel 만큼 가도록).
    """

    def __init__(self, axis="x", speed=0.1, travel=0.25, hold=1.5, scale=1.0, start_delay=2.0):
        self.ax = "xyz".index(axis[-1])
        self.sign = -1.0 if axis.startswith("-") else 1.0
        self.speed, self.travel, self.hold, self.scale = speed, travel, hold, scale
        self.start_delay = start_delay
        self.t0 = time.time()
        self.base = np.array([0.1, 0.2, -0.3])  # 임의의 Quest 손 위치

    def poll(self):
        t = time.time() - self.t0
        mode = "CONT" if t > 1.0 else "STOP"
        T = self.travel / self.speed
        tt = t - self.start_delay
        if tt < 0:
            s = 0.0
        elif tt < T:
            s = self.speed * tt
        elif tt < T + self.hold:
            s = self.travel
        elif tt < 2 * T + self.hold:
            s = self.travel - self.speed * (tt - T - self.hold)
        else:
            s = 0.0
        delta_robot = np.zeros(3)
        delta_robot[self.ax] = self.sign * s / self.scale
        wrist = self.base + R_QUEST_TO_ROBOT.T @ delta_robot
        self.wrist = wrist
        return wrist, mode, True

    def age(self):
        return 0.0

    def done(self):
        return time.time() - self.t0 > self.start_delay + 2 * self.travel / self.speed + self.hold + 1.0

    def close(self):
        pass
