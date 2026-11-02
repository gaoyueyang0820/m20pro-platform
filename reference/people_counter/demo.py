"""Real-time people counting in a plain desktop window."""
import argparse
import os
import time

import cv2
import numpy as np

from realtime_counter import RealtimeCounter, list_cameras, list_models

WIN = "People Counter"
FONT = cv2.FONT_HERSHEY_SIMPLEX


def draw_hud(frame, engine, show_help):
    s = engine.stats()
    h, w = frame.shape[:2]
    bar = 28
    cv2.rectangle(frame, (0, h - bar), (w, h), (0, 0, 0), -1)
    line = (f"{s['fps_infer']:.1f} FPS  {s['infer_ms']:.0f}ms  "
            f"IN {s['in_frame']}  TOTAL {s['total']}  PEAK {s['peak']}  "
            f"CONF {s['conf']:.2f}")
    cv2.putText(frame, line, (10, h - 9), FONT, 0.45, (200, 200, 200), 1,
                cv2.LINE_AA)

    if show_help:
        tip = "q quit  p pause  r reset  s snapshot  [ ] conf  f full  h help"
        tw = cv2.getTextSize(tip, FONT, 0.42, 1)[0][0]
        cv2.rectangle(frame, (w - tw - 16, h - bar - 26),
                      (w, h - bar), (0, 0, 0), -1)
        cv2.putText(frame, tip, (w - tw - 8, h - bar - 8), FONT, 0.42,
                    (150, 190, 255), 1, cv2.LINE_AA)

    if s["paused"]:
        cv2.putText(frame, "PAUSED", (w // 2 - 70, h // 2), FONT, 1.2,
                    (80, 80, 255), 2, cv2.LINE_AA)
    elif s["status"] != "ok":
        msg = (s["status_msg"] or s["status"])[:44]
        cv2.putText(frame, msg, (12, h // 2), FONT, 0.5, (100, 160, 255), 1,
                    cv2.LINE_AA)
    return frame


def no_signal(engine, w, h):
    img = np.full((h, w, 3), (18, 18, 20), np.uint8)
    cv2.putText(img, "NO SIGNAL", (30, h // 2 - 10), FONT, 1.0,
                (90, 90, 100), 2, cv2.LINE_AA)
    cv2.putText(img, (engine.status_msg or "waiting for camera")[:52],
                (30, h // 2 + 26), FONT, 0.5, (70, 70, 80), 1, cv2.LINE_AA)
    return img


def bring_to_front(title, topmost=False):
    """Windows-only: pull the OpenCV window to the foreground."""
    try:
        import ctypes
        u = ctypes.windll.user32
        hwnd = u.FindWindowW(None, title)
        if not hwnd:
            return
        u.ShowWindow(hwnd, 9)  # SW_RESTORE
        u.SetForegroundWindow(hwnd)
        if topmost:
            u.SetWindowPos(hwnd, -1, 0, 0, 0, 0, 0x0002 | 0x0001)  # TOPMOST|NOMOVE|NOSIZE
    except Exception:
        pass


def parse_args():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(
        description="Real-time people counter (desktop window)")
    p.add_argument("--source", default="0")
    p.add_argument("--model", default=os.path.join(here, "models", "yolo12n.pt"))
    p.add_argument("--conf", type=float, default=0.3)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--imgsz", type=int, default=480)
    p.add_argument("--torch-threads", type=int, default=12)
    p.add_argument("--min-frames", type=int, default=2)
    p.add_argument("--no-topmost", action="store_true")
    p.add_argument("--snapshot-dir", default=os.path.join(here, "output"))
    p.add_argument("--list", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()

    if args.list:
        cams = list_cameras()
        print("Cameras: " + (", ".join(map(str, cams)) if cams else "none found"))
        for m in list_models():
            print(f"  {m}")
        return

    engine = RealtimeCounter(
        source=args.source, model_path=args.model, conf=args.conf,
        width=args.width, height=args.height, imgsz=args.imgsz,
        torch_threads=args.torch_threads, min_frames=args.min_frames,
        publish=False,
    )
    engine.start()

    w, h = args.width, args.height
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, w, h)
    os.makedirs(args.snapshot_dir, exist_ok=True)
    print(f"window: {w}x{h}  model={os.path.basename(args.model)}  "
          f"imgsz={args.imgsz}  source={args.source}")
    print("keys: q quit | p pause | r reset | s snapshot | h help")

    show_help = True
    fullscreen = False
    shown_id, last_show = -1, 0.0
    first_shown = False

    try:
        while True:
            fid, frame = engine.latest_annotated()
            now = time.time()
            if frame is None:
                if now - last_show > 0.2:                # 没信号时也保持窗口刷新
                    cv2.imshow(WIN, no_signal(engine, w, h))
                    last_show = now
            elif fid != shown_id and now - last_show > 1 / 30:
                cv2.imshow(WIN, draw_hud(frame.copy(), engine, show_help))
                shown_id, last_show = fid, now
                if not first_shown:                      # 窗口是后台起的，主动提到最前
                    bring_to_front(WIN, topmost=not args.no_topmost)
                    first_shown = True

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("p"):
                engine.set_paused(not engine.stats()["paused"])
            elif key == ord("r"):
                engine.reset_counts()
            elif key == ord("s"):
                img = frame if frame is not None else no_signal(engine, w, h)
                path = os.path.join(args.snapshot_dir,
                                    time.strftime("shot_%Y%m%d_%H%M%S.jpg"))
                _, buf = cv2.imencode(".jpg", img,
                                      [int(cv2.IMWRITE_JPEG_QUALITY), 92])
                with open(path, "wb") as f:
                    f.write(buf.tobytes())
                print(f"saved {path}")
            elif key in (ord("["), ord("]")):
                delta = -0.05 if key == ord("[") else 0.05
                engine.set_conf(engine.stats()["conf"] + delta)
            elif key == ord("f"):
                fullscreen = not fullscreen
                cv2.setWindowProperty(
                    WIN, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
                if not fullscreen:
                    cv2.resizeWindow(WIN, w, h)
            elif key == ord("h"):
                show_help = not show_help

            if first_shown:
                try:
                    if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        engine.stop()
        cv2.destroyAllWindows()
        s = engine.stats()
        print(f"\nstopped. total={s['total']} peak={s['peak']}")


if __name__ == "__main__":
    main()