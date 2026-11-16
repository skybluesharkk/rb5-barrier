#!/usr/bin/env python3
"""Quest 앱 연결 진단 (로봇 없음). 패킷이 오는지, 초당 몇 개인지, 핀치 모드가 바뀌는지, 손목이 움직이는지를 1초마다 찍는다.

    python -m barrier.quest_check [--seconds 90] [--no-panel]

패널(15001 그래프, 10505 카메라)도 같이 보낸다. 구버전 APK 는 이 두 포트에 발행자가 없으면 예외를 초당 수백 번 던져 끊기므로,
--no-panel 과 비교하면 앱이 멈추는 원인이 패널 유무인지 알 수 있다.
"""
import argparse
import time

import numpy as np

from .quest import QuestReceiver


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--no-panel", action="store_true")
    a = ap.parse_args()
    q = QuestReceiver(a.host)
    panel = None
    if not a.no_panel:
        from .panel import PanelProxy
        panel = PanelProxy(a.host)
        panel.push(mode="check", alpha=0, scale=1.0, tracking=False, notice="quest_check running")
    print(f"듣는 중 {a.host}:8087/8095/8100 — 헤드셋을 쓰고 손을 움직여 보세요. (Ctrl+C 종료)", flush=True)
    t0 = time.time()
    last_n, last_mode, last_w = 0, None, None
    next_print = t0 + 1.0
    try:
        while time.time() - t0 < a.seconds:
            w, mode, fresh = q.poll()
            if mode != last_mode:
                print(f"[{time.time()-t0:6.1f}s] 모드 {last_mode} → {mode}", flush=True)
                last_mode = mode
            time.sleep(0.001)
            if time.time() >= next_print:
                next_print += 1.0
                n = q.n_packets
                moved = (np.linalg.norm(w - last_w) * 100) if (w is not None and last_w is not None) else 0.0
                print(f"[{time.time()-t0:6.1f}s] 패킷 {n-last_n:3d}/s  총 {n}  마지막 수신 {min(q.age(), 999):5.2f}s 전  "
                      f"손목 {np.round(w, 3) if w is not None else '-'}  1초 이동 {moved:.1f} cm  접두어:{q.last_prefix}  8100:{q.last_tel}", flush=True)
                last_n, last_w = n, (w.copy() if w is not None else None)
    except KeyboardInterrupt:
        pass
    finally:
        q.close()
        if panel:
            panel.close()


if __name__ == "__main__":
    main()
