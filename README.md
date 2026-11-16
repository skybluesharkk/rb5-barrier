# rb5-barrier

RB5-850E + Meta Quest(Open-Teach APK) 손 텔레옵에서 **박스 바리어(hard / CBF)에 닿을 때 로봇이 어떻게 서는지** 재는 실험 코드.
언리얼 없이 Quest → ZMQ → 이 PC → rbpodo(Wi-Fi) → RB5 로 바로 돈다.

- 사용법·실험 절차: [`barrier/README.md`](barrier/README.md)
- 작업 기록·결정 사항·Meta SDK 메모: [`barrier/HISTORY.md`](barrier/HISTORY.md)
- 설정: [`barrier/barrier.yaml`](barrier/barrier.yaml)
- Quest 앱 신뢰도 패치: [`barrier/quest_app_patch/`](barrier/quest_app_patch/)

```bash
conda activate dtrecon-capture
pip install -r requirements.txt
cd ~/rb5-barrier
python -m pytest barrier/tests
python -m barrier.teleop_barrier --mode hard --tag <이름>        # 실기
```

원래 [Jane](https://github.com/skybluesharkk/Jane) 저장소의 `server/barrier/` 로 시작했다가 2026-10-29 에 독립 저장소로 분리했다.
`barrier/logs/` 는 실행 기록(수십 MB)이라 git 에 넣지 않는다.
