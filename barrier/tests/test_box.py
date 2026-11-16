import numpy as np
import pytest

from barrier.box import Box, parse_box
from barrier.kinematics import fk, pose6_to_T

B = Box(np.array([0.0, -0.3, 0.2]), np.array([0.5, 0.1, 0.7]))
DT = 0.02


def test_distances_and_contains():
    d = B.distances([0.1, 0.0, 0.3])
    assert np.allclose(d, [0.1, 0.4, 0.3, 0.1, 0.1, 0.4])
    assert B.contains([0.1, 0.0, 0.3]) and not B.contains([0.6, 0.0, 0.3])


def test_none_passes():
    v, info = B.project([0.49, 0, 0.3], [1.0, 0, 0], DT, "none")
    assert np.allclose(v, [1.0, 0, 0]) and not info["active"].any()


def test_hard_stops_exactly_at_wall():
    p = np.array([0.495, 0.0, 0.3])
    v, info = B.project(p, [1.0, 0.0, 0.0], DT, "hard")      # 한 틱에 2 cm 가려 함, 벽까지 5 mm
    assert np.isclose(v[0], 0.005 / DT) and info["active"][1]
    assert np.isclose((p + v * DT)[0], 0.5)
    v, info = B.project([0.5, 0, 0.3], [1.0, 0, 0], DT, "hard")  # 벽 위: 법선 속도 0
    assert v[0] == 0.0 and info["active"][1]
    v, _ = B.project([0.5, 0, 0.3], [-1.0, 0, 0], DT, "hard")    # 벽에서 멀어지는 건 그대로
    assert v[0] == -1.0


def test_hard_untouched_far_from_wall():
    v, info = B.project([0.2, 0, 0.3], [0.5, 0.0, 0.0], DT, "hard")   # 1 cm 이동, 벽 30 cm
    assert np.allclose(v, [0.5, 0, 0]) and not info["active"].any()


def test_hard_outside_allows_return_only():
    v, _ = B.project([0.52, 0, 0.3], [1.0, 0, 0], DT, "hard")
    assert v[0] == 0.0
    v, _ = B.project([0.52, 0, 0.3], [-1.0, 0, 0], DT, "hard")
    assert v[0] == -1.0


def test_cbf_decelerates_before_wall():
    alpha = 5.0
    v, info = B.project([0.45, 0, 0.3], [1.0, 0, 0], DT, "cbf", alpha)  # d = 5 cm → v_n ≥ -0.25
    assert np.isclose(v[0], alpha * 0.05) and info["active"][1]
    # 반복 적분하면 벽을 넘지 않는다
    p = np.array([0.3, 0.0, 0.3])
    for _ in range(500):
        v, _ = B.project(p, [1.0, 0, 0], DT, "cbf", alpha)
        p = p + v * DT
    assert p[0] <= 0.5 + 1e-9 and p[0] > 0.49


def test_cbf_alpha_clamped_to_dt():
    v, info = B.project([0.495, 0, 0.3], [1.0, 0, 0], DT, "cbf", 1e6)
    assert np.isclose(v[0], 0.005 / DT) and np.isclose(info["alpha_eff"], 1 / DT)


def test_axes_independent():
    v, info = B.project([0.5, -0.3, 0.3], [1.0, -1.0, 0.3], DT, "hard")
    assert np.allclose(v, [0.0, 0.0, 0.3]) and info["active"][1] and info["active"][2]


def test_parse_box():
    b = parse_box("0,0.5,-0.3,0.1,0.2,0.7")
    assert np.allclose(b.lo, B.lo) and np.allclose(b.hi, B.hi)
    with pytest.raises(ValueError):
        parse_box("1,2,3")


def test_fk_matches_dtrecon_home_pose():
    # dt-recon-automation 가짜 로봇이 아니라 DH 자체 검사: 모든 관절 0 → 플랜지가 base 위쪽 어딘가에, 회전 정규직교
    T = fk([0, 0, 0, 0, 0, 0])
    R = T[:3, :3]
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)
    assert np.linalg.norm(T[:3, 3]) < 1.2
    # pose6 ↔ T 규약: ZYX
    T2 = pose6_to_T([100, 200, 300, 10, 20, 30])
    assert np.allclose(T2[:3, 3], [0.1, 0.2, 0.3])


def test_cylinder():
    from barrier.box import Cylinder
    c = Cylinder(0.25)
    assert np.isclose(c.distances([0.3, 0.4, 0.5])[0], 0.25)
    # 기둥 쪽으로 가는 속도: 벽 위에서 법선 성분만 0, 접선은 유지
    v, info = c.project([0.25, 0.0, 0.5], [-1.0, 0.3, 0.1], DT, "hard")
    assert np.allclose(v, [0.0, 0.3, 0.1]) and info["active"][0]
    # 멀어지는 건 그대로
    v, _ = c.project([0.25, 0.0, 0.5], [1.0, 0.0, 0.0], DT, "hard")
    assert np.allclose(v, [1.0, 0.0, 0.0])
    # cbf: 5 cm 앞에서 α·d 로 제한
    v, _ = c.project([0.30, 0.0, 0.5], [-1.0, 0.0, 0.0], DT, "cbf", 5.0)
    assert np.isclose(v[0], -0.25)
    # 반복 적분해도 안 들어감
    p = np.array([0.5, 0.1, 0.5])
    for _ in range(400):
        v, _ = c.project(p, [-1.0, -0.2, 0.0], DT, "hard")
        p = p + v * DT
    assert np.hypot(p[0], p[1]) >= 0.25 - 1e-9


def test_yaw_calibration():
    from barrier.quest import R_QUEST_TO_ROBOT as R0, hand_matrix, rot_z, yaw_to_align
    # Quest 추적 원점이 수직축으로 42° 틀어진 세계: 이상적 좌표 q_ideal(앞=+z) 가 실제로는 q_act 로 들어온다
    world = np.linalg.inv(R0) @ rot_z(42) @ R0
    fwd = world @ np.array([0, 0, 0.2])
    delta = R0 @ fwd                                   # 보정 전 로봇 좌표: +x 에서 42° 돌아감
    yaw = yaw_to_align(delta, np.array([1, 0, 0]))
    assert abs(yaw + 42) < 1e-9
    R = hand_matrix(yaw)
    assert np.allclose(R @ fwd, [0.2, 0, 0])                           # 앞 → 로봇 +x
    assert np.allclose(R @ (world @ [0, 1, 0]), [0, 0, 1])             # 위 → 로봇 z
    assert np.allclose(R @ (world @ [1, 0, 0]), [0, -1, 0])            # 오른쪽 → 사용자 오른쪽 (-y)
    assert yaw_to_align(np.array([0, -1, 0]), np.array([0, -1, 0])) == 0.0


def test_tracking_state_trailer():
    from barrier.quest import parse_keypoints, parse_tracking_state
    kps = "|".join("0.1,0.2,0.3" for _ in range(24))
    plain = f"absolute:{kps}:".encode()                       # 원본 APK (끝에 ':' 하나)
    assert parse_tracking_state(plain) is None
    pre, k = parse_keypoints(plain)
    assert pre == "absolute" and k.shape == (24, 3)
    patched = f"absolute:{kps}:1,1.000,1.000,0.750,0.000,1.000,1.000,1,1,123.4567".encode()
    pre, k = parse_keypoints(patched)
    assert pre == "absolute" and k.shape == (24, 3) and k[0, 2] == 0.3    # 키포인트 파싱은 영향 없음
    t = parse_tracking_state(patched)
    assert t["tracked"] == 1 and t["hand_conf"] == 1.0 and t["index_conf"] == 0.75 and t["middle_conf"] == 0.0 and t["high_conf"] == 1
    assert abs(t["unity_t"] - 123.4567) < 1e-9
    lost = f"absolute:{'|'.join('0,0,0' for _ in range(24))}:0,0.000,0.000,0.000,0.000,0.000,0.000,1,0,124.0".encode()
    assert parse_tracking_state(lost)["tracked"] == 0
