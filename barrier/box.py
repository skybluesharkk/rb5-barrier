"""축 정렬 박스 바리어. 순수 numpy — 로봇·네트워크 없이 시험 가능 (tests/test_box.py).

박스 면 6개: xmin, xmax, ymin, ymax, zmin, zmax. 각 면 i의 안쪽 법선 n_i, 면까지 거리 d_i (안쪽이면 양수).
속도 투영 규칙 (face마다 독립, 축 정렬이라 서로 간섭 없음):

    mode="none" : 그대로 통과 (안전 박스는 별도로 항상 hard)
    mode="hard" : f=0, 감속 항 없음. 벽에 닿은 뒤에만 법선 속도를 자른다.
                  이산 시간이라 "이번 틱에 벽을 넘지 않는" 것이 하드 제약: v·n_i ≥ -d_i/dt
    mode="cbf"  : 제어 바리어 함수. v·n_i ≥ -α·d_i  (벽에서 v/α 거리부터 감속 시작)
                  α·dt > 1 이면 한 틱에 벽을 넘을 수 있어 min(α, 1/dt)로 잡는다. α=∞ 가 곧 hard.
"""
from dataclasses import dataclass

import numpy as np

FACES = ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")
NAMES = FACES + ("rmin",)   # 기록·패널에 쓰는 제약 이름 7개 (박스 6면 + base 기둥 원기둥)
# 면 i: 축 번호, 부호 (안쪽 법선 = sign * e_axis)
_AXIS = (0, 0, 1, 1, 2, 2)
_SIGN = (1.0, -1.0, 1.0, -1.0, 1.0, -1.0)


@dataclass
class Box:
    lo: np.ndarray  # (3,) m
    hi: np.ndarray  # (3,) m

    @classmethod
    def from_dict(cls, d):
        return cls(np.array([d["xmin"], d["ymin"], d["zmin"]], float),
                   np.array([d["xmax"], d["ymax"], d["zmax"]], float))

    def to_dict(self):
        return dict(xmin=float(self.lo[0]), xmax=float(self.hi[0]), ymin=float(self.lo[1]),
                    ymax=float(self.hi[1]), zmin=float(self.lo[2]), zmax=float(self.hi[2]))

    def distances(self, p):
        """면 6개까지 거리 (m). 안쪽이면 양수, 넘었으면 음수. 순서는 FACES."""
        p = np.asarray(p, float)
        return np.array([p[0] - self.lo[0], self.hi[0] - p[0],
                         p[1] - self.lo[1], self.hi[1] - p[1],
                         p[2] - self.lo[2], self.hi[2] - p[2]])

    def contains(self, p, margin=0.0):
        return bool((self.distances(p) >= margin).all())

    def project(self, p, v, dt, mode="hard", alpha=np.inf):
        """속도 v (m/s)를 위치 p에서 바리어에 맞게 투영.

        반환: (v_new, info) — info = dict(d=거리6, active=면별 제약 작동 여부6, alpha_eff)
        """
        v = np.array(v, float)
        d = self.distances(p)
        active = np.zeros(6, bool)
        if mode == "none":
            return v, dict(d=d, active=active, alpha_eff=0.0)
        if mode == "hard":
            a_eff = 1.0 / dt
        elif mode == "cbf":
            a_eff = min(float(alpha), 1.0 / dt)
        else:
            raise ValueError(f"mode {mode!r}: none|hard|cbf")
        for i in range(6):
            ax, s = _AXIS[i], _SIGN[i]
            vn = s * v[ax]                       # 안쪽 법선 방향 속도 (양수 = 벽에서 멀어짐)
            floor = -a_eff * max(d[i], 0.0)      # 허용되는 최소 법선 속도
            if d[i] < 0:                          # 이미 넘어감: 벽 쪽으로 더 가지는 못함 (복귀는 허용)
                floor = 0.0
            if vn < floor:
                v[ax] = s * floor
                active[i] = True
        return v, dict(d=d, active=active, alpha_eff=a_eff)


def parse_box(spec):
    """'xmin,xmax,ymin,ymax,zmin,zmax' (m) 또는 dict → Box."""
    if isinstance(spec, Box):
        return spec
    if isinstance(spec, dict):
        return Box.from_dict(spec)
    vals = [float(x) for x in str(spec).split(",")]
    if len(vals) != 6:
        raise ValueError("box: xmin,xmax,ymin,ymax,zmin,zmax (m) 6개")
    return Box(np.array(vals[0::2]), np.array(vals[1::2]))


@dataclass
class Cylinder:
    """base z축 중심 수직 원기둥 *바깥* 만 허용 (기둥 특이점 회피). 거리 d = r_xy − r_min, 안쪽 법선 = 반지름 바깥 방향."""
    r_min: float

    def distances(self, p):
        p = np.asarray(p, float)
        return np.array([np.hypot(p[0], p[1]) - self.r_min])

    def project(self, p, v, dt, mode="hard", alpha=np.inf):
        p, v = np.asarray(p, float), np.array(v, float)
        d = self.distances(p)
        active = np.zeros(1, bool)
        if mode == "none":
            return v, dict(d=d, active=active, alpha_eff=0.0)
        a_eff = 1.0 / dt if mode == "hard" else min(float(alpha), 1.0 / dt)
        r = np.hypot(p[0], p[1])
        n = np.array([p[0] / r, p[1] / r, 0.0]) if r > 1e-9 else np.array([1.0, 0.0, 0.0])
        vn = v @ n
        floor = 0.0 if d[0] < 0 else -a_eff * d[0]
        if vn < floor:
            v = v + (floor - vn) * n
            active[0] = True
        return v, dict(d=d, active=active, alpha_eff=a_eff)
