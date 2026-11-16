"""Quest 그래프 패널(ZMQ PUB 15001)에 벽 거리 바를 그려 보낸다. 카메라 패널(10505)에는 검정 이미지 (빨간 테두리 제거).

matplotlib 렌더가 제어 루프의 GIL을 뺏어 틱 지터를 만들기 때문에 별도 프로세스에서 돈다.
제어 루프는 PanelProxy.push() 로 작은 dict 만 파이프에 넣는다.
"""
import io
import multiprocessing as mp
import time

import numpy as np

from .box import NAMES


def render(st: dict):
    import cv2
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    C = dict(bg="#1a1a2e", white="#ffffff", cyan="#00d4ff", green="#00ff88", gray="#888888", red="#ff4444", yellow="#ffd700")
    fig, ax = plt.subplots(figsize=(5, 4), dpi=100)
    fig.patch.set_facecolor(C["bg"])
    ax.set_facecolor(C["bg"])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    tx = ax.transAxes
    mode, alpha = st.get("mode", "?"), st.get("alpha", 0.0)
    ax.text(0.5, 0.97, f"Barrier teleop  —  {mode}" + (f"  α={alpha:g}" if mode == "cbf" else ""),
            ha="center", va="top", color=C["white"], fontsize=12, fontweight="bold", transform=tx)
    tracking = st.get("tracking", False)
    status = "● TRACKING  (Index pinch: stop)" if tracking else "○ STANDBY  (Middle pinch: start)"
    ax.text(0.5, 0.89, status, ha="center", va="top", color=C["green"] if tracking else C["gray"], fontsize=10, transform=tx)
    if st.get("notice"):
        ax.text(0.5, 0.82, st["notice"], ha="center", va="top", color=C["red"], fontsize=8, fontweight="bold", transform=tx)
    ax.text(0.05, 0.76, "Distance to wall  (robot cm  /  hand cm)", ha="left", va="top", color=C["cyan"], fontsize=9, transform=tx)
    d, active, scale = st.get("d"), st.get("active"), st.get("scale", 1.0)
    if d is not None:
        for i, name in enumerate(NAMES):
            if i >= len(d):
                break
            y = 0.68 - i * 0.075
            d_cm = d[i] * 100
            hit = active is not None and active[i]
            col = C["red"] if hit or d_cm <= 0 else (C["yellow"] if d_cm < 5 else C["green"])
            frac = float(np.clip(d_cm / 30.0, 0, 1))
            ax.add_patch(plt.Rectangle((0.30, y - 0.03), 0.55 * frac, 0.05, color=col, alpha=0.85, transform=tx))
            ax.text(0.05, y, name, ha="left", va="center", color=C["white"], fontsize=10, fontfamily="monospace", transform=tx)
            ax.text(0.87, y, f"{d_cm:5.1f} / {d_cm / max(scale, 1e-6):5.1f}", ha="left", va="center", color=col, fontsize=8,
                    fontfamily="monospace", transform=tx)
    else:
        ax.text(0.5, 0.5, "waiting for tracking", ha="center", va="center", color=C["gray"], fontsize=11, transform=tx)
    if st.get("tcp") is not None:
        t = st["tcp"]
        ax.text(0.05, 0.10, f"TCP  X {t[0]:+7.1f}  Y {t[1]:+7.1f}  Z {t[2]:+7.1f} mm", ha="left", va="center", color=C["white"],
                fontsize=9, fontfamily="monospace", transform=tx)
    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    buf.seek(0)
    img = cv2.imdecode(np.frombuffer(buf.read(), np.uint8), cv2.IMREAD_COLOR)
    _, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return jpg.tobytes()


def _cam_loop(sock, cam_index, stop, fps=15.0, width=640):
    """웹캠을 Quest 카메라 패널(10505)로. 앱이 내 시야를 다 가리므로 로봇을 비추는 카메라를 여기로 보낸다."""
    import cv2
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if not cap.isOpened():
        print(f"\n[panel] 카메라 {cam_index} 열기 실패 — 검은 화면으로 대체")
        cap = None
    _, blank = cv2.imencode(".jpg", np.zeros((360, 640, 3), np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 50])
    blank = blank.tobytes()
    period = 1.0 / fps
    while not stop.is_set():
        t = time.time()
        data = blank
        if cap is not None:
            ok, frame = cap.read()
            if ok:
                _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                data = jpg.tobytes()
        sock.send(data)
        time.sleep(max(0.0, period - (time.time() - t)))
    if cap is not None:
        cap.release()


def _panel_main(conn, host, graph_port, cam_port, hz, cam_index):
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C 는 부모가 처리, 자식은 파이프의 None 으로 종료
    import threading

    import zmq
    ctx = zmq.Context()
    g = ctx.socket(zmq.PUB)
    g.bind(f"tcp://{host}:{graph_port}")
    c = ctx.socket(zmq.PUB)
    c.bind(f"tcp://{host}:{cam_port}")
    stop = threading.Event()
    cam_th = threading.Thread(target=_cam_loop, args=(c, cam_index, stop), daemon=True)
    cam_th.start()
    st = {}
    period = 1.0 / hz
    try:
        while True:
            t = time.time()
            while conn.poll(0):
                msg = conn.recv()
                if msg is None:
                    return
                st.update(msg)
            try:
                g.send(render(st))
            except Exception as e:
                print(f"\n[panel] {e}")
            time.sleep(max(0.0, period - (time.time() - t)))
    finally:
        stop.set()
        cam_th.join(timeout=2.0)
        g.close(0)
        c.close(0)
        ctx.term()


class PanelProxy:
    """제어 루프 쪽 핸들. 속성을 바꾸고 push() 하면 패널 프로세스로 간다."""

    def __init__(self, host="0.0.0.0", graph_port=15001, cam_port=10505, hz=10.0, cam_index=-1):
        self.parent, child = mp.Pipe()
        self.proc = mp.Process(target=_panel_main, args=(child, host, graph_port, cam_port, hz, cam_index), daemon=True)
        self.proc.start()
        self.state = {}

    def push(self, **kw):
        clean = {k: (np.asarray(v).tolist() if isinstance(v, np.ndarray) else v) for k, v in kw.items()}
        self.state.update(clean)
        try:
            self.parent.send(clean)
        except Exception:
            pass

    def close(self):
        try:
            self.parent.send(None)
        except Exception:
            pass
        self.proc.join(timeout=2.0)
        if self.proc.is_alive():
            self.proc.terminate()
