# Quest 앱 패치: 손 추적 신뢰도 같이 보내기

대상: Open-Teach `VR/Franka-Bot-Unity/Assets/Scripts/Gesture Detection/GestureDetector.cs`
(https://github.com/aadhithya14/Open-Teach, 2024-03-12 Initial Commit 기준. 패키지 `com.Xigbee.FrankaBot`).

- `GestureDetector_confidence.patch` : 원본 대비 diff
- `GestureDetector.cs` : 패치 적용된 전체 파일 (덮어써도 됨)

## 바뀌는 것
`SendHandData` 가 보내는 문자열 끝(원래 `...x,y,z:` 로 끝남)에 필드 하나를 덧붙인다.

```
absolute:<24 키포인트>:tracked,handConf,thumb,index,middle,ring,pinky,dataValid,highConf,unityTime
```

| 값 | 출처 (OVRHand) | 의미 |
|---|---|---|
| tracked | `IsTracked` | HandTracked 상태 플래그 |
| handConf | `HandConfidence` | **0.000~1.000 실수**. SDK enum 은 Low=0, High=0x3f800000(=float 1.0 비트) 라 런타임이 주는 float 를 그대로 복원 |
| thumb..pinky | `GetFingerConfidence()` | 손가락별, 같은 방식의 실수 |
| dataValid | `IsDataValid` | SDK 호출 성공 여부 |
| highConf | `IsDataHighConfidence` | tracked && handConf High |
| unityTime | `Time.unscaledTime` | 앱 쪽 프레임 시각 (s) |

Open-Teach 의 `oculus.py` 는 `split(':')[1]` 만 쓰므로 그대로 호환. 이쪽 `barrier/quest.py` 는
`parse_tracking_state()` 로 읽어 `quest.csv` 의 `tracked, hand_conf, ..., unity_t` 열에 기록한다 (원본 APK 면 빈 칸).

## 빌드
1. Open-Teach 저장소 clone → Unity Hub 에서 `VR/Franka-Bot-Unity` 열기 (Unity 2021.3 LTS 계열, Android Build Support 필요.
   `ProjectVersion.txt` 가 없어서 버전은 패키지로 추정).
2. 위 파일을 `Assets/Scripts/Gesture Detection/GestureDetector.cs` 에 덮어쓰기.
3. File → Build Settings → Android → Build. 서명은 기본 debug keystore 면 된다.
4. `adb install -r <apk>` (기존 앱과 패키지 ID 가 같아서 그냥 덮어써짐. 앱 안 IP 설정은 PlayerPrefs 라 유지).

주의: 지금 Quest 에 설치된 `SingleArmBot_tactile_v9_with_camera.apk` 는 상류 소스 이후에 손본 빌드일 수 있다 (패널 수신 시
멈춤 문제가 고쳐져 있음). 상류 소스로 다시 빌드하면 그 수정이 빠질 수 있으니, 빌드 후 패널(`--no-panel` 없이) 동작을 한 번 확인할 것.

## 신뢰도 값에 대해
Meta 런타임은 신뢰도를 float 로 넘기고 Unity 래퍼는 양 끝만 `Low`/`High` 로 이름 붙인다 (`High = 0x3f800000` = `1.0f`).
패치는 그 비트를 float 로 다시 읽어 보내므로 런타임이 중간값을 내면 그대로 나온다. 다만 현재 펌웨어에서 0.0 과 1.0 외의 값이
실제로 나오는지는 확인된 바 없다 — 첫 실기 로그에서 `hand_conf` 열의 고유값을 확인할 것. 0/1 만 나온다면 연속값이 필요할 때
PC 쪽에서 (hand + 손가락 5개 평균) 또는 최근 N 프레임 High 비율로 0~1 을 만들면 된다 (`quest.csv` 에 재료가 다 있음).
