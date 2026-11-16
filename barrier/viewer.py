"""브라우저 3D 뷰어 (RViz 대용): 바리어 박스 + 팔 링크 + 손끝(측정/명령/손 목표) + 궤적을 실시간으로 본다.

제어 루프가 ViewerProxy.push() 로 상태를 넘기면 별도 프로세스의 HTTP 서버가 /state JSON 으로 내보내고,
viewer.html (three.js) 이 20 Hz 로 폴링해 그린다.  →  http://<이 PC IP>:8766

기록 재생:
    python -m barrier.viewer --replay logs/<run>            # 실시간 속도로 재생
    python -m barrier.viewer --replay logs/<run> --speed 0.25
"""
import argparse
import json
import multiprocessing as mp
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

from .box import NAMES
from .kinematics import urdf_frames

HERE = Path(__file__).parent


def _serve(conn, port):
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C 는 부모가 처리, 자식은 파이프의 None 으로 종료
    state = {"frames": None, "links": None,
             "meshes": sorted(p.name for p in (HERE / "meshes").glob("link*.dae")) if (HERE / "meshes").is_dir() else []}
    lock = threading.Lock()
    html = (HERE / "viewer.html").read_bytes()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            cache = "no-store"
            if self.path.startswith("/state"):
                with lock:
                    body = json.dumps(state).encode()
                ctype = "application/json"
            elif self.path.startswith("/meshes/"):
                f = HERE / "meshes" / Path(self.path).name
                if not f.is_file():
                    self.send_response(404)
                    self.end_headers()
                    return
                body, ctype, cache = f.read_bytes(), "model/vnd.collada+xml", "max-age=86400"
            else:
                body, ctype = html, "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache)
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        while True:
            msg = conn.recv()
            if msg is None:
                break
            q = msg.pop("jnt_ang", None)
            T_ft = msg.pop("T_flange_tcp", None)
            with lock:
                state.update(msg)
                if q is not None and np.isfinite(q).all():
                    F = urdf_frames(q)                      # link0..link6, 플랜지
                    if T_ft is not None:
                        F = F + [F[-1] @ np.asarray(T_ft)]  # + TCP
                    state["frames"] = [f[:3, 3].tolist() for f in F]
                    state["links"] = [f.ravel().tolist() for f in F[:7]]   # 메시용 4x4 (행 우선)
    finally:
        srv.shutdown()


class ViewerProxy:
    def __init__(self, port=8766):
        self.parent, child = mp.Pipe()
        self.proc = mp.Process(target=_serve, args=(child, port), daemon=True)
        self.proc.start()
        self.port = port

    def push(self, **kw):
        clean = {k: (np.asarray(v).tolist() if isinstance(v, np.ndarray) else v) for k, v in kw.items()}
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


def replay(run_dir, speed=1.0, port=8766):
    from .analyze import read_csv
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "meta.json").read_text())
    cmd, st = read_csv(run_dir / "cmd.csv"), read_csv(run_dir / "state.csv")
    v = ViewerProxy(port)
    v.push(box=meta["box"], safety_box=meta["safety_box"], r_min=meta.get("r_min", 0.0), mode=meta["mode"], alpha=meta["alpha"], scale=meta["scale"],
           replay=run_dir.name)
    print(f"재생 {run_dir.name}  http://localhost:{port}   (Ctrl+C 종료)")
    ti, tj = 0, 0
    t_start = time.time()
    try:
        while ti < len(cmd["t"]) or tj < len(st["t"]):
            now = (time.time() - t_start) * speed
            while tj < len(st["t"]) and st["t"][tj] <= now:
                v.push(jnt_ang=[st[f"jnt_ang_{i}"][tj] for i in range(6)],
                       tcp_meas=[st["tcp_meas_x"][tj], st["tcp_meas_y"][tj], st["tcp_meas_z"][tj]],
                       tcp_ref=[st["tcp_ref_x_mm"][tj] / 1000, st["tcp_ref_y_mm"][tj] / 1000, st["tcp_ref_z_mm"][tj] / 1000],
                       t=st["t"][tj])
                tj += 1
            while ti < len(cmd["t"]) and cmd["t"][ti] <= now:
                v.push(tracking=True, p_cmd=[cmd[f"p_cmd_{a}"][ti] for a in "xyz"], p_des=[cmd[f"p_des_{a}"][ti] for a in "xyz"],
                       d=[cmd[f"d_{f}"][ti] for f in NAMES if f"d_{f}" in cmd], active=[c == "1" for c in cmd["active"][ti]])
                ti += 1
            time.sleep(0.01)
        print("재생 끝. 창은 열려 있음 — Ctrl+C")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        v.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--replay", required=True, help="run 디렉터리")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--port", type=int, default=8766)
    a = ap.parse_args()
    replay(a.replay, a.speed, a.port)


if __name__ == "__main__":
    main()
