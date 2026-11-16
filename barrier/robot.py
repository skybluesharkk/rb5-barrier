"""로봇 입출력: 실제 RB5(rbpodo, Wi-Fi) 와 가짜 로봇. 같은 인터페이스.

  tcp_pose6()   현재 TCP (x,y,z [mm], rx,ry,rz [deg]) — 추적 시작 시 기준점
  servo(pose6)  move_servo_l 한 틱
  read_state()  dict(t, jnt_ang[6], jnt_ref[6], tcp_ref[6], tcp_meas[3 m], robot_state)
                tcp_meas = FK(jnt_ang, 엔코더) · T_flange_tcp → 정지 충격이 보이는 쪽
                tcp_ref  = 컨트롤 박스 지령값 (시뮬레이션 모드에서는 이것만 움직임)

StateLogger: 별도 스레드가 read_state()를 최대한 빨리 돌려 state.csv 에 쓴다 (제어 루프를 막지 않음).
"""
import csv
import threading
import time

import numpy as np

from .kinematics import fk, pose6_to_T

STATE_FIELDS = (["t", "robot_state"] + [f"jnt_ang_{i}" for i in range(6)] + [f"jnt_ref_{i}" for i in range(6)]
                + ["tcp_ref_x_mm", "tcp_ref_y_mm", "tcp_ref_z_mm", "tcp_ref_rx", "tcp_ref_ry", "tcp_ref_rz"]
                + ["tcp_meas_x", "tcp_meas_y", "tcp_meas_z"]
                + ["ems_flag", "collision", "self_collision", "soft_estop", "sos_flag"])   # 컨트롤러 상태 플래그
def _flag(v, hi):
    """컨트롤러 플래그. 이 펌웨어는 일부 필드(self_collision)에 쓰레기 값이 들어오므로 범위 밖이면 -1 (무시)."""
    v = int(v)
    return v if 0 <= v <= hi else -1


EMS_NAMES = {0: "", 1: "Arm Stretch (팔 완전히 뻗음/특이점)", 2: "Cartesian Limit", 3: "Joint Limit", 4: "Un-solvable (IK 해 없음)"}


class RB5Servo:
    def __init__(self, ip, speed_bar=0.3, simulation=False, t1=0.03, t2=0.1, gain=1.0, alpha=1.0, collision=True):
        self.ip, self.speed_bar, self.simulation = ip, speed_bar, simulation
        self.t1, self.t2, self.gain, self.alpha, self.collision = t1, t2, gain, alpha, collision
        self.T_flange_tcp = np.eye(4)

    def connect(self):
        import rbpodo as rb
        self.rb = rb
        self.rc = rb.ResponseCollector()
        self.robot = rb.Cobot(self.ip)
        self.data = rb.CobotData(self.ip)
        mode = rb.OperationMode.Simulation if self.simulation else rb.OperationMode.Real
        self.robot.set_operation_mode(self.rc, mode)
        self.robot.set_speed_bar(self.rc, self.speed_bar)
        self.robot.set_collision_onoff(self.rc, self.collision)
        self.robot.flush(self.rc)
        _, tfc = self.robot.get_tfc_info(self.rc)
        _, tcp = self.robot.get_tcp_info(self.rc)
        self.T_flange_tcp = np.linalg.inv(pose6_to_T(tfc)) @ pose6_to_T(tcp)
        self.rc = self.rc.error().throw_if_not_empty()
        s = self.read_state()
        if s is None:
            raise RuntimeError(f"{self.ip}: 상태 채널(5001) 응답 없음")
        if not self.simulation and np.abs(s["jnt_ref"] - s["jnt_ang"]).max() > 0.2:
            raise RuntimeError(f"{self.ip}: reference와 측정 관절각이 다름 — 시뮬레이션 모드로 움직인 뒤 실제로 전환됐을 수 있음. 펜던트로 확인")
        return self

    def tcp_pose6(self):
        _, tcp = self.robot.get_tcp_info(self.rc)
        return np.asarray(tcp, float).ravel()

    def servo(self, pose6):
        self.robot.move_servo_l(self.rc, np.asarray(pose6, float), self.t1, self.t2, self.gain, self.alpha)

    def read_state(self):
        d = self.data.request_data(0.2)
        if d is None:
            return None
        s = d.sdata
        q = np.array(s.jnt_ang, float).ravel()
        tcp_meas = (fk(q) @ self.T_flange_tcp)[:3, 3]
        return dict(t=time.time(), robot_state=int(s.robot_state), jnt_ang=q, jnt_ref=np.array(s.jnt_ref, float).ravel(),
                    tcp_ref=np.array(s.tcp_ref, float).ravel(), tcp_meas=tcp_meas,
                    flags=tuple(_flag(v, hi) for v, hi in ((s.op_stat_ems_flag, 4), (s.op_stat_collision_occur, 1),
                                                           (s.op_stat_self_collision, 1), (s.op_stat_soft_estop_occur, 1),
                                                           (s.op_stat_sos_flag, 20))))

    def stop(self):
        try:
            self.robot.task_stop(self.rc)
        except Exception:
            pass

    def close(self):
        try:
            self.robot.flush(self.rc)
        except Exception:
            pass


class FakeRobot:
    """Cartesian 2차 지연 모델. servo 목표 → 참조(1차, t1) → 실제(2차 임계감쇠, ω=2/t2). 관절각은 NaN."""

    def __init__(self, start_pose6=(300.0, -150.0, 450.0, 180.0, 0.0, 90.0), t1=0.03, t2=0.1, rate=500.0, **_):
        self.pose6 = np.array(start_pose6, float)
        self.t1, self.t2, self.rate = t1, t2, rate
        self.p_cmd = self.pose6[:3] / 1000.0
        self.p_ref = self.p_cmd.copy()
        self.x = self.p_cmd.copy()
        self.xd = np.zeros(3)
        self.lock = threading.Lock()
        self.run = True
        self.th = threading.Thread(target=self._loop, daemon=True)

    def connect(self):
        self.th.start()
        return self

    def _loop(self):
        dt = 1.0 / self.rate
        wn = 2.0 / self.t2
        while self.run:
            with self.lock:
                self.p_ref += (self.p_cmd - self.p_ref) * min(1.0, dt / self.t1)
                acc = wn * wn * (self.p_ref - self.x) - 2.0 * wn * self.xd
                self.xd += acc * dt
                self.x += self.xd * dt
            time.sleep(dt)

    def tcp_pose6(self):
        with self.lock:
            return np.concatenate([self.x * 1000.0, self.pose6[3:]])

    def servo(self, pose6):
        with self.lock:
            self.p_cmd = np.asarray(pose6, float)[:3] / 1000.0

    def read_state(self):
        with self.lock:
            ref = np.concatenate([self.p_ref * 1000.0, self.pose6[3:]])
            return dict(t=time.time(), robot_state=1, jnt_ang=np.full(6, np.nan), jnt_ref=np.full(6, np.nan),
                        tcp_ref=ref, tcp_meas=self.x.copy(), flags=(0, 0, 0, 0, 0))

    def stop(self):
        pass

    def close(self):
        self.run = False


class StateLogger:
    """read_state()를 스레드로 돌려 state.csv 기록. latest 로 마지막 상태를 공유."""

    def __init__(self, robot, path, t0, rate_hz=100.0):
        self.robot, self.path, self.t0, self.period = robot, path, t0, 1.0 / rate_hz
        self.latest = None
        self.n = 0
        self.n_fail = 0
        self.run = True
        self.th = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.th.start()
        return self

    def _loop(self):
        with open(self.path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(STATE_FIELDS)
            while self.run:
                tick = time.perf_counter()
                try:
                    s = self.robot.read_state()
                except Exception:
                    s = None
                if s is None:
                    self.n_fail += 1
                else:
                    self.latest = s
                    self.n += 1
                    w.writerow([f"{s['t'] - self.t0:.4f}", s["robot_state"], *[f"{v:.4f}" for v in s["jnt_ang"]],
                                *[f"{v:.4f}" for v in s["jnt_ref"]], *[f"{v:.3f}" for v in s["tcp_ref"]],
                                *[f"{v:.6f}" for v in s["tcp_meas"]], *s["flags"]])
                rem = self.period - (time.perf_counter() - tick)
                if rem > 0:
                    time.sleep(rem)

    def stop(self):
        self.run = False
        self.th.join(timeout=2.0)
