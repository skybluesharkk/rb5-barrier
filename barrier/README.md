# barrier — 박스 바리어 정지 거동 실험 (Quest 텔레옵, 언리얼 없음)

사용자 손(Meta Quest)으로 RB5 손끝을 끌고 가다가 박스 바리어에 닿을 때 로봇이 **쿵 하고 서는지, 부드럽게 서는지**를
숫자(위치·속도·가속도 시간 그래프)와 눈으로 판단하기 위한 도구.

```
Quest (Open-Teach APK) --ZMQ 8087 (Wi-Fi)--> 이 PC  --rbpodo move_servo_l (Wi-Fi 192.168.50.5x)--> RB5
                        <--ZMQ 15001 패널--      │
                                                 ├--> http://<PC>:8766  브라우저 3D 뷰어 (박스·팔·손끝·궤적)
                                                 └--> logs/<run>/  cmd.csv, state.csv, meta.json → analyze.py
```

## 설치

```bash
conda activate dtrecon-capture       # rbpodo 가 있는 환경 (dt-recon-automation 과 같이 씀)
pip install pyzmq pyyaml             # 나머지(opencv, matplotlib, scipy)는 이미 있음
cd ~/rb5-barrier
python -m pytest barrier/tests       # 바리어 수식 검사
```

Quest 쪽: Open-Teach APK 가 설치돼 있어야 하고, 앱이 접속할 PC IP 를 이 PC 로 (`ip -br -4 addr`).
Wi-Fi 절전은 꺼져 있어야 한다 (`iw dev wlp3s0 get power_save` → off).

## 하드웨어 없이 먼저

```bash
python -m barrier.teleop_barrier --fake-robot --fake-quest 0.05 --mode hard --tag hard_slow     # 손 5 cm/s
python -m barrier.teleop_barrier --fake-robot --fake-quest 0.20 --mode hard --tag hard_fast     # 손 20 cm/s
python -m barrier.teleop_barrier --fake-robot --fake-quest 0.20 --mode cbf --alpha 5 --tag cbf_fast
python -m barrier.analyze barrier/logs                 # → logs/barrier_compare.png + .csv
python -m barrier.viewer --replay barrier/logs/<run>   # 기록을 3D 로 재생
```

Quest 는 있고 로봇은 없을 때: `--fake-robot` 만. 로봇은 있고 Quest 가 없을 때: `--fake-quest`.
Jane 저장소의 `~/Jane/server/mock_quest.py --host 127.0.0.1` 로 ZMQ 경로 자체를 시험할 수도 있다.

## 실제 로봇

1. 펜던트/대시보드로 팔을 실험 박스(`barrier.yaml` `box`) 안, 벽에서 10 cm 이상 떨어진 곳에 둔다. 박스 밖이면 실행을 거부한다.
2. `--sim` 으로 먼저 (컨트롤러 시뮬레이션 모드, 팔 안 움직임, tcp_ref 만 움직임).
3. 실제:
   ```bash
   python -m barrier.teleop_barrier --robot-ip 192.168.50.50 --mode hard --tag hard_slow
   ```
   Middle pinch → 추적 시작 (그 순간 손·로봇 위치가 기준점), Index pinch → 정지, Ctrl+C → 종료.
   **좌표계 보정**: 추적 시작 직후 로봇은 멈춰 있다. 손을 로봇 쪽(앞)으로 15 cm 이상 쭉 뻗으면 `[보정 완료] yaw …°` 가 뜨고
   그때부터 로봇이 따라온다. Quest 추적 원점의 yaw 는 앱 시작·리센터마다 달라서 "앞"이 로봇 어느 축인지 매번 다르기 때문이다
   (2026-10-23 로그: 앞으로 뻗은 손이 로봇 +x·+y 사이로 가서 "왼쪽으로 간다"고 보임). `barrier.yaml` `hand_frame.user_forward`
   가 사용자가 바라보는 로봇 방향 (기본 +x). 보정을 끄려면 `--calib-dist 0 --yaw <deg>`.
   **손 추적이 끊기면** (2026-10-27 현재 아무 처리 없음, `barrier.yaml` `quest:` 참고): 점프가드·자동 재시작·패킷 끊김 재기준은 꺼져 있고,
   Quest 가 손을 못 볼 때 보내는 (0,0,0) 만 무시한다 (`ignore_zero`). 시야에서 사라졌다 돌아올 때의 거동을 실기로 보고 새 해결책을 넣을 예정.
   사용자가 손을 **천천히**(실험 1) / **다소 빠르게**(실험 2) 한 벽으로 밀고, 벽 너머에서 1~2 초 계속 민 뒤 돌아온다.
   조건마다 3회 이상 반복.
4. 비교 조건: `--mode hard` (f=0, 감속 없음) 와 `--mode cbf --alpha 5` (대조군). `--t2` (컨트롤러 스무딩)를 바꿔 보면
   "부드러움"이 바리어에서 오는지 로봇 필터에서 오는지 갈라진다.
5. `python -m barrier.analyze logs/<run1> logs/<run2> ...`

## 제어 (한 틱)

```
p_des = p_init + scale · Rz(yaw) · R_quest→robot · (wrist − wrist_init)     yaw: 보정값 (meta.json yaw_deg)
v_des = clip((p_des − p_cmd)/dt, v_max)
v_cmd = box.project(p_cmd, v_des)     hard: v·n ≥ −d/dt (벽을 넘지 않는 것만)   cbf: v·n ≥ −α·d
        → safety_box.project(hard)    항상
p_cmd += v_cmd·dt  →  move_servo_l(p_cmd, 시작 자세 고정)
```
hard 모드에서 손이 벽 밖에 머무는 동안 p_cmd 는 벽에 붙어 있고, 손이 박스 안으로 돌아와야 다시 움직인다 (벽을 미는 상황 재현).

## 기록 (logs/<날짜시각>_<tag>/)

| 파일 | 주기 | 내용 |
|---|---|---|
| quest.csv | Quest 패킷마다 (~70 Hz) | t, prefix(absolute=팔 모드/relative=손만), 손목 Quest 좌표, 점프가드 통과 여부 |
| cmd.csv | 제어 루프 (50 Hz, 대기 중에도) | t, dt, tracking, mode, 손목(Quest 좌표), hand(로봇 좌표), p_des, v_des, v_cmd, p_cmd, 제약 7개 거리, 작동 플래그, **tcp_meas / tcp_ref**(그 시점 로봇, m), state_age |
| state.csv | 상태 스레드 (≤100 Hz, 네트워크에 따라) | jnt_ang(엔코더), jnt_ref(지령), tcp_ref(지령 mm), **tcp_meas = FK(jnt_ang)·T_flange_tcp** |
| meta.json | — | 조건, 박스, r_min, 루프 dt 통계 |

한 줄로 "그 순간 손이 어디 있었고 로봇이 어디 있었나"를 보려면 cmd.csv 한 파일이면 된다 (hand_* 와 tcp_meas_*). 더 촘촘한 로봇 궤적은 state.csv.

analyze.py 는 tcp_meas 로 벽 거리 s, 접근 속도 v=−ds/dt, 가속도 a=dv/dt 를 구하고 (SG 필터 미분),
바리어가 처음 작동한 순간을 t=0 으로 맞춰 조건별로 겹친다. 표: 접근 속도, 최대 감속, 최대 저크, 벽 넘은 거리, 정지까지 시간.

## 실제 로봇에서 무엇을 재야 하나

- **엔코더 관절각(jnt_ang)** 으로 손끝을 계산한다. `tcp_ref`/`jnt_ref` 는 컨트롤 박스의 지령값이라 정지 충격·외력이 안 보인다.
  rb5-teleop 의 `/joint_states` 도 jnt_ref 를 읽으므로 같은 함정이 있다.
- 50 Hz 차분을 두 번 하면 노이즈뿐이다. 가속도는 필터 미분으로 보되, 충격 자체는 **플랜지에 IMU(스마트폰이라도)** 를 붙여 직접 재는 게 가장 확실하다.
- Wi-Fi 지터는 `meta.json` 의 dt_max/p99 와 `state_fail` 로 확인. 심하면 랜선(Jane 은 10.0.2.x 유선 별도망을 썼다).
- 손 자체의 접근 속도(cmd.csv 의 v_des)를 같이 봐야 "느릴 때 부드럽다"가 바리어 덕인지 그냥 느려서인지 구분된다.

## 예상 (가짜 로봇 결과, logs/barrier_compare.png)

hard 는 벽에 닿는 순간 법선 속도를 한 틱에 0 으로 만든다 → 감속 피크가 접근 속도에 비례 (5 cm/s: −0.3 m/s², 20 cm/s: −1.2 m/s²).
cbf 는 v/α 거리부터 미리 줄여 같은 속도에서 피크가 절반 (−0.7 m/s²), 대신 정지까지 더 오래 걸린다.
"느리면 부드럽다"는 바리어가 잘해서가 아니라 자를 속도가 작아서다.

## 파일

| | |
|---|---|
| `box.py` | 박스 바리어 수식 (none/hard/cbf), 순수 numpy |
| `quest.py` | Open-Teach APK ZMQ 수신 (servo_teleop.py 에서 분리) + FakeQuest |
| `robot.py` | RB5Servo (rbpodo, Wi-Fi) / FakeRobot, 상태 기록 스레드 |
| `kinematics.py` | RB5 DH 순기구학 (dt-recon-automation 사본) |
| `panel.py` | Quest 그래프 패널(15001)에 벽 거리 바 — 별도 프로세스 |
| `viewer.py`, `viewer.html` | 브라우저 3D 뷰어 + 기록 재생 |
| `mqtt.py` | 선택: Jane 언리얼용 topic/robot, topic/barrier 발행 (`--mqtt broker`) |
| `teleop_barrier.py` | 메인 루프 |
| `analyze.py` | 그래프·요약표 |
