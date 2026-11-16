# 바리어 실험 작업 기록 (2026-10-23 ~ 2026-10-27)

Quest 텔레옵 + RB5-850E 로 "CBF 에서 감속 없이 배리어만 있을 때(f=0) 로봇이 어떻게 멈추는가"를 측정하기 위해
원래 Jane 저장소 `~/Jane/server/barrier/` 안에서 만들고 실기까지 돌린 과정의 정리 (2026-10-29 에 `~/rb5-barrier/barrier/` 독립 저장소로 옮김). 코드 사용법은 `README.md`, 설정은 `barrier.yaml`.

---

## 1. 목표와 설계 결정

**실험 목표**: 사용자 손(Meta Quest 손 추적)이 박스 밖으로 나가려 할 때 로봇 손끝(TCP)이 벽에서 어떻게 멈추는지
(쿵 vs 부드럽게)를 천천히/빠르게 접근 × hard/cbf 조건으로 기록하고, 거리·속도·가속도 그래프와 수치로 비교.

**구조 결정**
- 언리얼(Jane) 은 중계 역할뿐이라 **빼고**, Open-Teach Quest APK → ZMQ → 이 PC(Python) → rbpodo(Wi-Fi) → 로봇 으로 직결.
  rb5-teleop 은 Windows 언리얼 PC 가 손 추적 소스라 쓰지 않음. Jane 의 `servo_teleop.py` 수신·핀치 로직을 가져옴.
- 이 PC 하나 + Quest + 로봇이면 됨. 사용자에게 보여 줄 화면은 PC 브라우저 3D 뷰어(three.js, RB5 URDF 메시)로 대체.
- 바리어는 속도 수준 투영: hard `v·n ≥ -d/dt`, cbf `v·n ≥ -α·d`, none 없음. 안전 박스는 항상 hard.
- 측정 TCP 는 컨트롤러 참조값(`tcp_ref`)이 아니라 **엔코더 관절각 FK** (`jnt_ang` → DH FK · T_flange_tcp). 참조값은 정지 충격을 보여 주지 않음.
- 환경: `conda activate dtrecon-capture`, 로그는 `barrier/logs/<날짜시각>_<tag>/`. git 브랜치 `barrier-experiment`, **아직 커밋 안 함**.

**파일**
`box.py`(박스·원기둥 제약), `kinematics.py`(DH FK, URDF 프레임), `quest.py`(수신·가짜 손), `robot.py`(rbpodo 서보·상태 로거·가짜 로봇),
`panel.py`(Quest 패널 송신, 별도 프로세스), `viewer.py`+`viewer.html`(브라우저 3D), `mqtt.py`(선택), `teleop_barrier.py`(메인 50 Hz 루프),
`analyze.py`(그래프·표), `quest_check.py`(진단), `tests/test_box.py`(13개), `quest_app_patch/`(APK 패치), `barrier.yaml`.

---

## 2. 설정 변경 이력 (`barrier.yaml`)

| 항목 | 처음 | 현재 | 비고 |
|---|---|---|---|
| 박스 x (앞뒤) | Jane WORKSPACE ×1.5, base 중심 (±0.3375) | **±0.75** | 10-27 "앞뒤도 똑같이" |
| 박스 y (좌우) | ±0.3375 → ±0.45 → ±0.75 | **±0.55** | 10-27 "좌우 55 cm" |
| 박스 z | 0.20 ~ 0.95 | 그대로 | 아래는 책상 |
| 안전 박스 | 실험 박스 + 각 면 0.10 (아래 0.05) | x ±0.85, y ±0.65, z 0.15~1.05 | 항상 hard, 빨간색 |
| 원기둥 r_min | 0.25 | **0.15** | "너무 넓다" |
| scale | 1.0 → 0.7 | **1.0** | 손 10 cm = 로봇 10 cm |
| v_max | 0.25 m/s | 그대로 | 빠른 실험에서 0.35 정도로 올릴 수 있음 |
| speed_bar / t1 / t2 | 0.3 / 0.03 / 0.10 | 그대로 | |
| 손 좌표 yaw | 없음 | **-42° 고정** (`hand_frame.yaw_deg`) | 자동 보정은 `calib_dist: 0` 으로 꺼 둠 |
| quest.jump_guard_m | 0.10 (servo_teleop) | **0 (끔)** | 10-27 |
| quest.ignore_zero | 없음 | **true** | (0,0,0) 패킷 무시 |
| quest.stall_restart | 없음 | **false** | |
| quest.timeout_restart | 0.5 s 후 재기준 (servo_teleop) | **false** | |

주의: RB5-850E 도달 반경 ≈ 0.9 m 라 수평 0.75 m 벽은 z 낮을 때만 닿음. 먼 모서리는 바리어보다 팔 길이가 먼저 막음.

---

## 3. Quest 앱 관련 (10-23)

- 앱 메뉴: 왼손 **약지 핀치** → 메뉴(빨강) → Change IP → Stream. 왼손 **중지** = 팔 모드(파랑, `absolute`), **검지** = 손만(초록, `relative`),
  **새끼** = 해상도(검정). 오른손이 추적 대상. IP 는 PlayerPrefs 에 저장.
- 원본 `SingleArmBot.apk` 는 패널을 받으면 멈춤 → 포크의 `SingleArmBot_tactile_v9_with_camera.apk`(패키지 `com.Xigbee.FrankaBot`) 로
  adb 로 교체. udev 규칙(vendor 2833) 추가. 교체 후 크래시 없어짐.
- v9 빌드는 8100 포트(모드)로 아무것도 안 보냄 → 모드를 키포인트 접두어(`absolute`/`relative`)로 판정.
- IP 충돌: .49 는 다른 장비(MAC fc:22:1c:40:76:75)가 쓰고 있었음 → PC .30 사용. (`sudo ip addr del 192.168.50.49/24 dev wlp3s0` 권장)
- 패킷 ~70 Hz, 24 키포인트, 0번 손목, Unity 좌표(x 오른쪽, y 위, z 앞). 신뢰도 없음.

---

## 4. 좌표계 문제 (10-27)

- 증상: 손을 앞으로 뻗으면 플랜지가 왼쪽으로 감.
- 로그 분석: 위아래는 맞고, 수평만 로봇 +x 에서 +y 쪽으로 약 42° 틀어짐. 홈포즈·관절각과 무관 (명령은 base 좌표 Cartesian).
- 원인 후보: 로봇 base 설치 방향 + Quest 추적 원점(가디언) 방향의 합. 둘 다 상수 → 고정 yaw 로 흡수.
- 레퍼런스 확인: Jane `servo_teleop.py`, rb5-teleop 모두 고정 행렬만 씀, 보정 없음. 처음에 "앱 켤 때마다 바뀐다"고 한 건 과한 표현이었음
  (리센터·가디언 재설정 때만 바뀜).
- 구현: `Rz(yaw)·R_quest→robot`. 자동 보정(추적 시작 후 손을 앞으로 15 cm 뻗으면 yaw 계산) 코드는 있으나 `calib_dist: 0` 으로 꺼 둠.
- 미확인: 이후 실행 두 번에서 손의 주 이동 방향이 보정 후에도 +40° 로 나옴. "앞"으로만 밀었다면 `--yaw -84` 가 맞을 수 있음.
  사용자는 "오프셋은 대충 괜찮다"고 함. 사용자가 base +x 를 보고 서 있다는 가정(`user_forward`)도 미확인.

---

## 5. 실기 실행과 발견

### 10-23 (17:35, 18:34, 18:45) hard
- 첫 실기. 로봇 추종 확인. 좌표계 틀어짐 발견의 근거 로그.

### 10-27 15:02 `yawcheck`, 15:21 `hard_1`
- `hard_1`: 벽이 아닌데 로봇이 멈춤. 바리어 접촉 0회. **손목 특이점**: TCP 를 +x 로 50 cm 가져가는 동안 J1 90°→183°, J4 82°→2°, J6 148°→83°,
  J5 → -175°. 컨트롤러 참조값이 고정되고 서보 거부. 15.6 s 에 Idle.
- 대응: 컨트롤러 플래그(ems_flag: Arm Stretch/Joint Limit 등, collision, pause)를 `state.csv` 에 기록. 시작 시 관절각 출력 + J5 특이점 25° 이내 /
  수평 반경 70 cm 이상이면 경고. "못 따라옴" 감지(참조와 명령 3 cm 이상 0.5 s) → 경고 한 번.
- 권장: 시작 전 J5 ≈ -90°, 팔을 밀 방향으로 미리 돌려 놓기.

### 10-27 15:38 `1027_01_hard`, 15:50 `02_updown`, 15:58 `03_updown`
- 고개를 돌려 시야에서 손을 없애도 로봇이 따라감 → **Quest 는 시야 밖에서도 손을 계속 추적** (패킷 정상, 동결 0%). 정상 동작.
- 카메라를 손으로 가림 → Quest 가 **(0,0,0) 을 계속 전송**. servo_teleop 식 점프가드가 1 s 뒤 (0,0,0) 을 "새 좌표계"로 수용 →
  목표 z = -0.86 m → 로봇이 v_max 로 급강하, **실험 박스 바닥(zmin 0.20)이 잡음**. 손 복귀 시 또 점프 수용 → 못 따라옴 경고 반복
  (내가 넣은 정지 처리가 다음 틱에 자동 재시작하는 결함).

### 10-27 16:52 `1027_hard_4_out_of_sight` (처리 전부 끈 상태)
- 가리는 순간 (0,0,0) 사이에 **유령 패킷 2개** (실제 손에서 27 cm 떨어진 좌표) → 점프가드 없음 → 그 좌표가 목표 →
  로봇이 0.25 m/s 로 26 cm 상승해 5.5 s 유지 → 손 복귀 시 다시 하강.
- 원인: Open-Teach 앱이 `IsTracked`/`HandConfidence` 를 안 보고 OVRSkeleton 뼈 위치를 매 프레임 그대로 전송. 놓치기 직전 1~2 프레임의
  저신뢰 외삽 자세가 유령 패킷, 완전히 놓치면 RootPose 원점 → (0,0,0).

---

## 6. 손 추적 끊김 처리의 변천 (사용자 요청 반영)

1. 내가 "정지 + Index→Middle 재핀치만 재시작" 으로 바꿈 → 사용자: "마음대로 한 것, 원복".
2. servo_teleop 방식(1 s 후 수용, 자동 재시작)으로 원복.
3. 사용자: "없다고 생각하고 실기로 보고 새 해결책 만들 것" → **전부 끔** (`quest:` 스위치). 패킷 끊김 후 재기준(servo_teleop 원본 기능)도
   사용자 요청으로 끔. 남은 것은 `(0,0,0) 무시` 하나.

**지금 설계** (각 경우)
- 패킷 끊김: 경고 한 번, 마지막 목표 유지, 재개되면 원래 기준점으로 이어감.
- (0,0,0): 무시, 마지막 유효 좌표 유지 (`ignore_zero: false` 면 그대로 사용 → 바닥 벽까지 내려감).
- 다른 위치에서 재등장(유령 포함): 검사 없이 즉시 목표, v_max 로 이동.
- 못 따라옴: 경고 한 번, 추적 유지.
- 어떤 경우든 실험 박스·안전 박스·원기둥은 걸림. Index 핀치 정지, Middle 핀치 재시작(그 순간 위치가 기준).

---

## 6-1. Meta SDK 가 앱에 주는 손 추적 데이터 (Open-Teach 프로젝트에 포함된 Oculus Integration 기준)

**패킷에는 없고 SDK 안에는 있는 것** — `OVRPlugin.HandState` (매 프레임, `OVRPlugin.GetHandState`)

| 필드 | 내용 |
|---|---|
| `Status` 플래그 | `HandTracked`(추적 중), `InputStateValid`, `SystemGestureInProgress`, `DominantHand` |
| `HandConfidence` | 손 전체 신뢰도. `TrackingConfidence { Low = 0, High = 0x3f800000 }` |
| `FingerConfidences[5]` | 손가락별 신뢰도, 같은 enum |
| `RootPose` | 손목 위치 + 회전 (Posef) |
| `BoneRotations[24]` | 뼈마다 부모 기준 회전. 위치는 OVRSkeleton 이 회전에서 계산 |
| `HandScale` | 손 크기 배율 |
| `Pinches`, `PinchStrength[5]` | 핀치 비트마스크, 세기 0~1 |
| `PointerPose` | 레이 포인터 자세 |
| `RequestedTimeStamp`, `SampleTimeStamp` | 샘플 시각 (Open-Teach 는 안 씀) |

- `0x3f800000` 은 IEEE-754 float `1.0` 의 비트 패턴 → 네이티브 런타임은 신뢰도를 float 로 넘기고 C# 래퍼가 양 끝만 이름 붙인 것.
  비트를 float 로 다시 읽으면 0~1 실수. 현재 펌웨어가 0/1 외 중간값을 실제로 내는지는 **미확인** (첫 실기 로그에서 확인할 것).
- HandState 에 float 신뢰도 필드는 따로 없음. OpenXR `XR_EXT_hand_tracking` 도 관절별 valid/tracked 비트뿐, 실수 신뢰도 없음.

**Unity 래퍼 `OVRHand`** (`Assets/Oculus/VR/Scripts/Util/OVRHand.cs`)
- `IsTracked`, `HandConfidence`, `IsDataValid`(GetHandState 성공 여부), `IsDataHighConfidence`(= IsTracked && High),
  `GetFingerConfidence(finger)`, `GetFingerIsPinching`, `GetFingerPinchStrength`, `PointerPose`, `HandScale`, `IsSystemGestureInProgress`.
- 호출 실패 시 기본값: HandConfidence Low, HandScale 1.0, 핀치 0.

**`OVRSkeleton`** (`OVRSkeleton.cs`)
- 손 뼈 24개: `Hand_WristRoot`(0), `Hand_ForearmStub`, Thumb0~3, Index1~3, Middle1~3, Ring1~3, Pinky0~3, 손끝 5개(Thumb/Index/Middle/Ring/PinkyTip).
- `Update()`: `IsDataValid` 가 false 면 아무것도 안 함(뼈는 마지막 자세 유지). 유효하면 RootPose 를 루트에, BoneRotations 를 각 뼈 localRotation 에 적용.
- 우리 로그의 (0,0,0): 손을 완전히 놓치면 런타임이 RootPose 를 원점(identity)으로 주고 IsDataValid 는 true 라 손목이 추적 원점 = (0,0,0) 이 됨(추정, 로그와 일치).
  유령 패킷: HandTracked 아직 켜진 채 신뢰도 Low 로 외삽된 1~2 프레임.

**Open-Teach 앱이 실제로 하는 일** (`Assets/Scripts/Gesture Detection/GestureDetector.cs`)
- `SendHandData`: `RightHandSkeleton.Bones` 의 `Transform.position` 24개를 `x,y,z|...` 로 직렬화, 접두어 `absolute:`/`relative:`, 끝에 `:` 하나.
  PUSH 소켓으로 8087 에 매 Update 프레임 전송. **IsTracked/HandConfidence 를 한 번도 읽지 않음.** 왼손은 핀치 감지에만 사용.
- 수신 `openteach/components/detector/oculus.py`: `split(':')[1]` 을 `|` 와 `,` 로 나눠 점마다 앞 3개 값만 사용. 신뢰도·유효성 검사 없음.
- 포트: 8087 키포인트, 8095 해상도 버튼, 8100 일시정지/모드, 15001 그래프 패널(PC→Quest), 10505 카메라 패널, 15002 택타일.

**raw 값을 PC 에서 보는 방법**: Quest 는 손 추적을 앱 밖으로 내보내지 않음(adb logcat 에도 없음). 앱이 읽어서 보내야 함 →
(1) Open-Teach Unity 프로젝트 수정(아래 7절), (2) rb5-teleop 언리얼 `VRUser.cpp` 의 QuestHandSampleCapture(Windows 언리얼 PC 필요),
(3) 현재 APK 그대로는 불가.

## 7. Quest 앱 패치 (`quest_app_patch/`)

- Open-Teach 저장소에 Unity 소스가 **있음** (`VR/Franka-Bot-Unity`, 6,214 파일, 앱 코드는 `Assets/Scripts/` 9개). `VR/APK/` 는 빌드본.
  설치된 v9 와 패키지 ID 동일. Unity 2021.3 LTS 추정(ProjectVersion.txt 없음), Oculus Integration·NetMQ 포함.
- 패치: `GestureDetector.cs` `SendHandData` 끝에 필드 추가
  `absolute:<키포인트>:tracked,handConf,thumb,index,middle,ring,pinky,dataValid,highConf,unityTime`
  신뢰도 6개는 **0.000~1.000 실수**: SDK enum `High = 0x3f800000` 이 float 1.0 비트라 비트를 float 로 복원. 펌웨어가 중간값을 내는지는 미확인.
- 수신: `quest.py` `parse_tracking_state()` 가 읽어 `quest.csv` 끝 열에 기록 (원본 APK 면 빈 칸). 키포인트 파싱 영향 없음. Open-Teach Python 과도 호환.
- 주의: v9 빌드는 상류 이후 손본 것일 수 있음 → 상류 소스로 빌드하면 패널 수정이 빠질 수 있으니 빌드 후 확인.

---

## 8. 로그 형식

| 파일 | 주기 | 내용 |
|---|---|---|
| `quest.csv` | 패킷마다 (~70 Hz) | t, prefix, 손목 Quest 좌표, accepted, (패치 APK) tracked·신뢰도 6개·data_valid·high_conf·unity_t |
| `cmd.csv` | 틱마다 (50 Hz) | 손목, hand(로봇 좌표 목표), p_des, v_des, v_cmd, p_cmd, 제약 7개 거리, active, safety_active, tcp_meas, tcp_ref, state_age |
| `state.csv` | ~90 Hz | robot_state, jnt_ang, jnt_ref, tcp_ref, tcp_meas(FK), ems_flag, collision, self_collision(쓰레기값이면 -1), soft_estop, sos_flag |
| `meta.json` | 1회 | 설정 전부 (박스, yaw, quest 스위치, 서보 파라미터, 시작 TCP) |

분석: `python -m barrier.analyze barrier/logs/<run> ...` (접촉 기준 정렬, 거리/접근속도/가속도 그래프, 표).

---

## 9. 실행 방법

```
cd ~/rb5-barrier
conda activate dtrecon-capture
python -m barrier.teleop_barrier --robot-ip 192.168.50.50 --mode hard --no-panel --tag <이름>
```
- 시작 줄에 `점프가드 끔, (0,0,0) 무시 True, 못따라옴 자동재시작 False, 패킷끊김 자동재시작 False` 와 관절각 경고 확인.
- `--no-panel`: Quest 안 패널(거리 막대·카메라) 송신 끔. 시야 가림 때문에 유지 중. 빼면 패널, `--cam 0` 이면 웹캠.
- 종료 Ctrl+C. 뷰어는 브라우저 자동 오픈(`--no-browser` 로 끔).
- 비교 조건: `--mode cbf --alpha 5`, 빠른 접근은 `--v-max 0.35` 정도.

---

## 10. 남은 것 / 미확인

- 손 추적 끊김(유령 패킷·(0,0,0)·복귀)에 대한 **새 해결책 설계** — 패치 APK 로 신뢰도 받아서 설계하는 방향.
- yaw 가 -42 로 충분한지 (앞으로 20 cm → 제자리 → 오른쪽 20 cm 패턴으로 한 번 더 확인하면 정확히 계산 가능).
- cbf 조건 실기 아직 없음. slow/fast × hard/cbf 본 실험 미실시.
- 커밋 안 됨 (사용자 요청 시에만). `.49` 제거, 5 GHz Wi-Fi 권장.
