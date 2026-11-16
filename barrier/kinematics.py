"""RB5-850E 순기구학 (dt-recon-automation의 configs/rb5_kinematics.json 사본, 2026-10-16 양팔 291쌍으로 맞춘 DH).

측정 관절각(jnt_ang, 엔코더)으로 손끝 위치를 계산하기 위한 것. 컨트롤 박스가 주는 tcp_ref는 지령값이라
외력·정지 충격이 안 보인다 → 가속도 분석은 FK(jnt_ang)로 한다.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

_DH = json.loads((Path(__file__).parent / "rb5_kinematics.json").read_text())


def _dh(th, d, a, al):
    ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])


def frames(q_deg):
    """base → 각 관절 프레임 7개 (마지막 = TFC 플랜지). 뷰어의 팔 링크 그리기용."""
    q = np.radians(np.asarray(q_deg, float).ravel())
    M = np.eye(4)
    out = [M]
    for i in range(6):
        M = M @ _dh(q[i] + _DH["offset"][i], _DH["d"][i], _DH["a"][i], _DH["alpha"][i])
        out.append(M)
    return out


def fk(q_deg):
    """관절각 6개 [deg] → 플랜지(TFC) 포즈 4x4 [m]."""
    return frames(q_deg)[-1]


def pose6_to_T(p):
    """rbpodo (x,y,z [mm], rx,ry,rz [deg]) → 4x4 [m]. 규약 ZYX (R = Rz·Ry·Rx), dt-recon에서 실제 컨트롤러로 확정."""
    p = np.asarray(p, float).ravel()
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYX", [p[5], p[4], p[3]], degrees=True).as_matrix()
    T[:3, 3] = p[:3] / 1000.0
    return T


# rbpodo_ros2 rbpodo_description/robots/rb5_850e_u.urdf 의 관절 체인 (origin xyz [m], 회전축). 시각 메시(meshes/linkN.dae)는 링크 프레임 그대로.
# rbpodo 관절각(deg)을 부호 변환 없이 넣으면 위 DH FK 와 2.5 mm / 0.06° 안에서 일치 (2026-10-23 검증).
URDF_JOINTS = [((0, 0, 0.1655), (0, 0, 1)), ((0, 0, 0), (0, 1, 0)), ((0, 0, 0.425), (0, 1, 0)),
               ((0, 0, 0.392), (0, 1, 0)), ((0, -0.1107, 0.1107), (0, 0, 1)), ((0, 0, 0), (0, 1, 0))]
URDF_TCP = (0, -0.0967, 0)


def urdf_frames(q_deg):
    """base → link0..link6 프레임 7개 + TFC(플랜지) = 8개 4x4 [m]. 뷰어의 메시 배치용."""
    q = np.radians(np.asarray(q_deg, float).ravel())
    T = np.eye(4)
    out = [T.copy()]
    for (xyz, ax), qi in zip(URDF_JOINTS, q):
        A = np.eye(4)
        A[:3, 3] = xyz
        R = np.eye(4)
        R[:3, :3] = Rotation.from_rotvec(np.asarray(ax, float) * qi).as_matrix()
        T = T @ A @ R
        out.append(T.copy())
    E = np.eye(4)
    E[:3, 3] = URDF_TCP
    out.append(T @ E)
    return out
