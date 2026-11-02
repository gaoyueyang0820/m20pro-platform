#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DEEP Robotics Lynx M20 Pro —— Web 控制台（脱离手柄）

跑在你的 Windows PC 上（PC 需与机器人同网段）：
    python m20_console.py                  # 默认机器人 IP 10.21.31.103
    python m20_console.py --ip 10.21.41.1  # 按实际网络改
然后浏览器打开  http://127.0.0.1:8080/

协议依据：《Lynx M20 Series Software Development Manual》V0.0.4-0 巡检通信协议
  帧 = 16 字节协议头 + JSON ASDU
  头 = 4B 同步(EB 91 EB 90) + 2B 长度(小端) + 2B 消息ID(小端) + 1B 格式(=1 JSON) + 7B 保留
  ASDU = {"PatrolDevice":{"Type":t,"Command":c,"Time":"%Y-%m-%d %H:%M:%S","Items":{...}}}
    - 心跳  Type=100  Command=100  (≥1Hz；机器人把状态回报到发信方 IP:端口)
    - 姿态  Type=2    Command=22   Items.MotionParam  (0=空闲 1=站立 2=软急停 3=上电阻尼 4=坐 6=标准)
    - 步态  Type=2    Command=23   Items.GaitParam    (1=Basic 14=Stair)
    - 行走  Type=2    Command=21   Items.{X,Y,Z,Roll,Pitch,Yaw} 各 [-1,1]
            X=前后 Y=左右 Yaw=转向；Basic/Stair 下仅 X/Y/Yaw 生效；建议 20Hz；常规/辅助模式有效
    - 模式  Type=1101 Command=5    Items.Mode         (0=常规 1=导航 2=辅助)
            辅助模式(=2)下轴指令照常下发，底层自动叠加避障修正；避障状态见 BasicStatus.OOA
    - 状态  回包 Type=1002 Command=6 (基础 @2Hz) / Command=4 (运动 @10Hz)
仅依赖 Python 标准库，无需 pip install。

安全：操作前把狗放平地或吊架，物理急停放手边。先看状态在线再动作。
      行走指令走"保持 + 失联归零"机制：前端 0.5s 不上报速度即自动归零。
"""
import argparse
import json
import os
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

try:
    import cv2                      # 用于把 RTSP 转成 MJPEG；缺失时视频功能自动禁用
except ImportError:
    cv2 = None

try:
    import numpy as np              # 安全检测需要（YOLO 输出是 numpy）
except ImportError:
    np = None

try:
    from ultralytics import YOLO    # 安全检测模型；缺失时安全检测自动禁用
except ImportError:
    YOLO = None

# ---------------- 协议常量 ----------------
SYNC = b"\xEB\x91\xEB\x90"      # 0xEB + 外部/内部标志 + 0xEB + 0x90
ASDU_FORMAT_JSON = 0x01

# ---------------- 安全检测（PPE）常量 ----------------
# 模型路径：默认从用户的 Construction-PPE-Detection 项目里读
PPE_MODEL_PATH = os.environ.get(
    "M20_PPE_MODEL",
    r"D:\1\1\Construction-PPE-Detection-main\Model\ppe.pt"
)
PPE_CONF = float(os.environ.get("M20_PPE_CONF", "0.5"))
PPE_INTERVAL = float(os.environ.get("M20_PPE_INTERVAL", "0.15")) # 每 N 秒抓一帧检测（原 0.5，现 0.15 ≈ 7fps）
PPE_PERSIST = float(os.environ.get("M20_PPE_PERSIST", "2.0"))    # 持续 N 秒才报警
PPE_COOLDOWN = float(os.environ.get("M20_PPE_COOLDOWN", "10.0")) # 报警冷却

# 违规类型 → 中文
PPE_VIOLATION_CN = {
    "NO-Hardhat": "未戴安全帽",
    "NO-Mask": "未戴口罩",
    "NO-Safety Vest": "未穿安全背心",
}

# 类别颜色 (BGR)，与原项目一致
PPE_CLASS_COLORS = {
    0: (255, 0, 0),     # Hardhat 蓝
    1: (0, 255, 0),     # Mask 绿
    2: (0, 0, 255),     # NO-Hardhat 红
    3: (255, 255, 0),   # NO-Mask 青
    4: (255, 0, 255),   # NO-Safety Vest 品红
    5: (0, 255, 255),   # Person 黄
    6: (128, 0, 128),   # Safety Cone 紫
    7: (128, 128, 0),   # Safety Vest 橄榄
    8: (0, 128, 128),   # Machinery 暗青
    9: (128, 128, 128), # Vehicle 灰
}

MP = {"idle": 0, "stand": 1, "estop": 2, "damping": 3, "lie": 4, "standard": 6}
# 步态枚举（GaitParam，钉钉《软件开发指南》，公开手册 V0.0.4-0 里的 1/14 是旧值）：
#   标准运动模式 (0x100X)：基础=0x1001  高台=0x1002
#   敏捷运动模式 (0x300X)：平地=0x3002  楼梯=0x3003
# Gait 状态回包是整数值（如 4097 = 0x1001），下面也提供反向映射用于前端翻译
GAIT = {
    "basic":     0x1001,   # 基础（标准运动模式）
    "platform":  0x1002,   # 高台（标准运动模式）
    "stair":     0x3003,   # 楼梯（敏捷运动模式）
}
GAIT_REVERSE = {v: k for k, v in GAIT.items()}
GAIT_CN = {
    "basic": "基础步态",
    "platform": "高台步态",
    "stair": "楼梯步态",
}

# 相机（软件开发手册 Appendix 2）：前/后广角 RTSP 流
CAM = {"front": "video1", "rear": "video2"}
RTSP_PORT = 8554

VTYPE, VCMD = 2, 21             # 行走（轴指令），手册 1.2.5
VEL_HZ = 20                     # 手册建议 20Hz
VEL_FAILSAFE = 0.5              # 超过此秒数没收到前端速度 → 归零


def _clamp(v, lo=-1.0, hi=1.0):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    return max(lo, min(hi, v))


# ---------------- UDP 链路 ----------------
class M20Link:
    def __init__(self, ip, port=30000, hb_hz=3.0):
        self.ip, self.port, self.hb_hz = ip, port, hb_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", 0))          # 临时源端口；机器人会回报到这个端口
        self.sock.settimeout(0.2)
        self.lock = threading.Lock()
        self._seq = 0
        self.snapshots = {}                     # Command 上报码 -> Items
        self.last_rx = 0.0
        self.last_tx = 0.0
        self.log = []
        self._last_rxlog = 0.0
        self._stop = False
        # 行走保持状态
        self._vel = {"f": 0.0, "s": 0.0, "y": 0.0}
        self._vel_ts = 0.0                      # 前端最后一次上报速度的时间
        self._vel_last_nz = 0.0                 # 最后一次非零的时间
        self._vel_logged = False
        threading.Thread(target=self._hb_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._vel_loop, daemon=True).start()

    # ---- 日志 ----
    def _log(self, kind, text):
        with self.lock:
            self.log.append({"t": time.strftime("%H:%M:%S"), "kind": kind, "text": text})
            del self.log[:-200]

    # ---- 组包 ----
    def apdu(self, type_, command, items=None):
        with self.lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
        body = json.dumps(
            {"PatrolDevice": {"Type": type_, "Command": command,
                              "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                              "Items": items or {}}},
            separators=(",", ":")).encode("utf-8")
        if len(body) > 65535:
            raise ValueError("ASDU 过长")
        header = SYNC + struct.pack("<HH", len(body), seq) + bytes([ASDU_FORMAT_JSON]) + b"\x00" * 7
        return header + body

    def send(self, buf, label=""):
        try:
            self.sock.sendto(buf, (self.ip, self.port))
            self.last_tx = time.time()
            if label:
                self._log("tx", label)
            return True
        except OSError as e:
            self._log("err", "发送失败: %s" % e)
            return False

    # ---- 指令 ----
    def heartbeat(self):
        self.send(self.apdu(100, 100))

    def set_motion(self, key):
        # damping 已从前端移除，但后端保留命令以兼容旧客户端
        if key not in MP:
            return False
        cn = {"stand": "站起", "lie": "趴下", "estop": "软急停", "damping": "阻尼",
              "idle": "空闲", "standard": "标准站立"}.get(key, key)
        return self.send(self.apdu(2, 22, {"MotionParam": MP[key]}), "姿态 → %s" % cn)

    # 记住最后一次前后灯状态，以便半开关时补齐两个字段（机器人可能要求 Front/Back 同时存在）
    _front_light = 0
    _back_light = 0

    # 前进朝向：False=正向(头朝前)  True=反向(尾朝前)
    # 巡检协议里没有方向切换指令，所以通过翻转速度指令的 X 轴符号实现
    # 用户体验完全等价——推"前"摇杆时，reverse=False 头向前走，reverse=True 尾向前走
    _reverse = False

    def set_flashlight(self, front_on=None, back_on=None):
        """控制前后手电筒（Type=1101 Command=2）。
        机器人可能要求 Front/Back 两个字段同时存在，所以每次都把两个都发出去。
        只传一个灯时，另一个沿用上次状态（默认初始都是关）。"""
        if front_on is not None:
            self._front_light = 1 if front_on else 0
        if back_on is not None:
            self._back_light = 1 if back_on else 0
        items = {"Front": self._front_light, "Back": self._back_light}
        f_desc = "开" if items["Front"] == 1 else "关"
        b_desc = "开" if items["Back"] == 1 else "关"
        return self.send(self.apdu(1101, 2, items), "手电筒 → 前=%s 后=%s" % (f_desc, b_desc))

    def set_direction(self, forward):
        """切换前进朝向。
        巡检协议无此指令，通过翻转速度指令 X 轴符号实现。
        forward=True  → 正向（头朝前走）
        forward=False → 反向（尾朝前走）"""
        with self.lock:
            self._reverse = not forward
        self._log("tx", "朝向切换 → %s" % ("正向(头前)" if forward else "反向(尾前)"))
        return True

    def set_gait(self, key):
        if key not in GAIT:
            return False
        cn = GAIT_CN.get(key, key)
        return self.send(self.apdu(2, 23, {"GaitParam": GAIT[key]}),
                         "步态 → %s (GaitParam=0x%04x)" % (cn, GAIT[key]))

    def set_mode(self, m):
        m = int(m)
        return self.send(self.apdu(1101, 5, {"Mode": m}),
                         "使用模式 → %s" % {0: "常规", 1: "导航", 2: "辅助"}.get(m, m))

    def usage_mode(self):
        """从基础状态回包读当前使用模式（0=常规 1=导航 2=辅助），读不到返回 None。"""
        with self.lock:
            items = self.snapshots.get("6") or {}
        return (items.get("BasicStatus") or {}).get("ControlUsageMode")

    def _send_axis(self, f, s, y):
        # 方向翻转：reverse=True 时，把前后速度 X 取反
        with self.lock:
            rev = self._reverse
        if rev:
            f = -f
        items = {"X": round(f, 3), "Y": round(s, 3), "Z": 0.0,
                 "Roll": 0.0, "Pitch": 0.0, "Yaw": round(y, 3)}
        return self.send(self.apdu(VTYPE, VCMD, items))   # 20Hz 不打日志，避免刷屏

    def set_velocity(self, f, s, y):
        """前端调用：写目标速度（后端按 20Hz 保活下发）。"""
        f, s, y = _clamp(f), _clamp(s), _clamp(y)
        nz = abs(f) + abs(s) + abs(y) > 0.001
        with self.lock:
            self._vel = {"f": f, "s": s, "y": y}
            self._vel_ts = time.time()
            if nz:
                self._vel_last_nz = self._vel_ts
        # 仅在"模式未知"时兜底切常规；已知模式下不强行覆盖用户手选的模式
        if nz and self.usage_mode() is None:
            self.set_mode(0)
        return True

    # ---- 后台线程 ----
    def _hb_loop(self):
        period = 1.0 / max(0.5, self.hb_hz)
        while not self._stop:
            self.heartbeat()
            time.sleep(period)

    def _vel_loop(self):
        """20Hz 下发当前速度；0.5s 无前端更新则自动归零（失联保护）。"""
        period = 1.0 / VEL_HZ
        while not self._stop:
            time.sleep(period)
            now = time.time()
            with self.lock:
                v = dict(self._vel)
                ts = self._vel_ts
                nz = abs(v["f"]) + abs(v["s"]) + abs(v["y"]) > 0.001
            # 失联保护：目标非零但前端很久没上报 → 归零
            if nz and now - ts > VEL_FAILSAFE:
                v = {"f": 0.0, "s": 0.0, "y": 0.0}
                with self.lock:
                    self._vel = v
                self._log("err", "行走指令超时（前端失联），已归零")
                nz = False
            # 只在"运动中"或"刚停下 0.4s 内"下发，空闲时完全不打扰机器人
            if nz or now - self._vel_last_nz < 0.4:
                self._send_axis(v["f"], v["s"], v["y"])
            # 日志只在启动/停止时打一条
            if nz and not self._vel_logged:
                self._vel_logged = True
                self._log("tx", "行走开始 X=%.2f Y=%.2f Yaw=%.2f" % (v["f"], v["s"], v["y"]))
            elif not nz and self._vel_logged:
                self._vel_logged = False
                self._log("tx", "行走停止")

    def _rx_loop(self):
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            msg = self._parse(data)
            if not msg:
                continue
            pd = msg.get("PatrolDevice", {})
            cmd, items = pd.get("Command"), pd.get("Items")
            if cmd is None:
                continue
            self.last_rx = time.time()
            with self.lock:
                self.snapshots[str(cmd)] = items
            now = time.time()
            if now - self._last_rxlog > 2.0:      # 节流，避免刷屏
                self._last_rxlog = now
                self._log("rx", "状态回报 cmd=%s" % cmd)

    @staticmethod
    def _parse(data):
        if len(data) < 16:
            return None
        length = struct.unpack("<H", data[4:6])[0]
        body = data[16:16 + length]
        try:
            return json.loads(body.decode("utf-8"))
        except Exception:
            return None

    def payload(self):
        now = time.time()
        with self.lock:
            snaps = dict(self.snapshots)
            log = self.log[-40:]
            vel = dict(self._vel)
            rev = self._reverse
        basic = (snaps.get("6") or {}).get("BasicStatus") or {}
        motion = (snaps.get("4") or {}).get("MotionStatus") or {}
        return {
            "ip": self.ip, "port": self.port,
            "online": bool(self.last_rx and now - self.last_rx < 2.0),
            "rx_age": round(now - self.last_rx, 1) if self.last_rx else None,
            "basic": basic, "motion": motion,
            "cmd_vel": vel,
            "reverse": rev,
            "snapshots": snaps,
            "log": log,
        }


# ---------------- PPE 安全检测器 ----------------
class PPEDetector:
    """
    基于 YOLO 的安全帽/口罩/安全背心检测。
    独立线程从机器人 RTSP 流抓帧（不复用 MJPEG 通道，避免画面卡顿），
    每 PPE_INTERVAL 秒抓一帧跑推理。违规结果：
      - 画框写进 self._annotated（MJPEG 通道叠加显示）
      - 累计成 self.events，前端通过 /api/ppe/status 轮询
    """
    def __init__(self, cam="front"):
        self.cam = cam                       # "front" / "rear"，对应 CAM 字典
        self.enabled = False
        self.model = None
        self._stop = False
        self._thread = None
        self._cap = None
        self.lock = threading.Lock()
        self.last_hardhat_time = time.time() # 上次看到 Hardhat 的时刻
        self.last_alert_time = 0.0           # 上次报警时刻
        self.events = []                     # 最近告警（最多 50 条）
        self.last_dets = []                  # 最近一次的检测框（画 MJPEG 用）
        self.last_dets_time = 0.0
        self.last_frame_age = None           # 距上次成功抓帧的秒数
        self.last_frame_time = 0.0
        self.last_infer_ms = None
        self.error = None                    # 最近的错误信息（前端展示）
        self._annotated = None               # 最近一次画过框的 JPEG 字节
        self.last_frame = None               # 抓帧线程存的最新原始帧（YOLO 用）
        self.last_frame_time = 0.0
        self._grab_count = 0
        self._grab_thread = None
        self.last_annotated = None           # YOLO 直接画好框的推流尺寸帧（_video_ppe 读它）
        self.last_annotated_time = 0.0

    # ---- 模型加载 ----
    def _load_model(self):
        if self.model is not None:
            return True
        if YOLO is None:
            self.error = "ultralytics 未安装"
            return False
        if not os.path.isfile(PPE_MODEL_PATH):
            self.error = "模型文件不存在: %s" % PPE_MODEL_PATH
            return False
        try:
            self.model = YOLO(PPE_MODEL_PATH)
            # warmup：拿 dummy 帧跑一次，吃掉首次推理的 JIT 编译开销（2-3s）
            import numpy as _np
            dummy = _np.zeros((480, 640, 3), dtype=_np.uint8)
            try:
                self.model(dummy, conf=PPE_CONF, verbose=False)
            except Exception:
                pass
            self.error = None
            return True
        except Exception as e:
            self.error = "模型加载失败: %s" % e
            return False

    # ---- 主循环：YOLO 线程 + 抓帧线程，各干各的，互不干扰 ----
    # 【漂移根因】之前没 .copy()：cap.read() 复用同一个 OpenCV 内部缓冲区，
    #   YOLO 还在推理上一帧时，grab 已经把缓冲区写了新帧 → 半新半旧的幽灵帧 → 框飘
    # 【修复】grab 线程存 last_frame 时加 .copy()，彻底脱离 OpenCV 内部缓冲
    # 【流畅】_video_ppe 读 YOLO 已画好框的帧 → 全速 JPEG 编码推流（和 YOLO 并行）
    def _run(self):
        if not self._load_model():
            with self.lock:
                self.enabled = False
            return
        LINK._log("tx", "PPE 启动 GPU_check=%s imgsz=320" % ("OK" if self._check_gpu() else "CPU"))
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;tcp|buffer_size;1024|max_delay;0|"
            "fflags;nobuffer|flags;low_delay|avioflags;direct"
        )
        stream = CAM[self.cam]
        url = "rtsp://%s:%d/%s" % (LINK.ip, RTSP_PORT, stream)
        # 启动抓帧线程（带 .copy()！）
        threading.Thread(target=self._grab_loop, args=(url,), daemon=True).start()
        # YOLO 主循环：推理 → 画框 → 存 annotated 帧
        while not self._stop:
            try:
                with self.lock:
                    frame = self.last_frame
                if frame is None:
                    time.sleep(0.02); continue
                h, w = frame.shape[:2]
                disp = cv2.resize(frame, (640, int(h * 640 / w))) if w > 640 else frame.copy()
                t0 = time.time()
                dets = self._infer_one(disp)
                infer_ms = (time.time() - t0) * 1000.0
                annotated = self._draw(disp.copy(), dets) if dets else disp
                now = time.time()
                with self.lock:
                    self.last_dets = dets
                    self.last_dets_time = now
                    self.last_infer_ms = round(infer_ms, 1)
                    self.last_annotated = annotated
                    self.last_annotated_time = now
                    self.error = None
                self._check_violation(dets, disp)
            except Exception as e:
                with self.lock:
                    self.error = "检测异常: %s" % e
                time.sleep(1.0)

    def _grab_loop(self, url):
        """抓帧线程。关键：存 .copy()，不引用 OpenCV 内部缓冲。"""
        cap = None
        while not self._stop:
            try:
                if cap is None:
                    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                    try: cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception: pass
                    if not cap.isOpened():
                        cap.release(); cap = None
                        with self.lock: self.error = "RTSP 打不开: %s" % url
                        time.sleep(1.5); continue
                ok, frame = cap.read()
                if not ok or frame is None:
                    cap.release(); cap = None
                    time.sleep(0.3); continue
                # .copy() 是关键！脱离 OpenCV 内部缓冲，防止 YOLO 还在用就被覆盖
                with self.lock:
                    self.last_frame = frame.copy()
                    self.last_frame_time = time.time()
                    self.error = None
            except Exception as e:
                with self.lock: self.error = "抓帧异常: %s" % e
                time.sleep(0.5)
        if cap is not None: cap.release()

    def _check_gpu(self):
        try:
            import torch
            return torch.cuda.is_available()
        except Exception:
            return False

    def _infer_one(self, frame):
        """对一帧跑 YOLO，返回 dets 列表。"""
        try:
            results = self.model(frame, conf=PPE_CONF, verbose=False, imgsz=320)
        except Exception:
            return []
        dets = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls = int(box.cls[0])
                name = self.model.names[cls]
                dets.append({
                    "class_id": cls,
                    "class_name": name,
                    "confidence": float(box.conf[0]),
                    "x1": int(box.xyxy[0][0]),
                    "y1": int(box.xyxy[0][1]),
                    "x2": int(box.xyxy[0][2]),
                    "y2": int(box.xyxy[0][3]),
                })
        return dets

    # ---- 违规判定（沿用原 ViolationChecker 思路，但简化） ----
    def _check_violation(self, dets, frame):
        now = time.time()
        hardhat = any(d["class_name"] == "Hardhat" for d in dets)
        person = any(d["class_name"] == "Person" for d in dets)
        if hardhat:
            self.last_hardhat_time = now
        if not (person and not hardhat):
            return
        time_without = now - self.last_hardhat_time
        time_since_alert = now - self.last_alert_time
        if time_without < PPE_PERSIST or time_since_alert < PPE_COOLDOWN:
            return
        self.last_alert_time = now
        # 找出 person 框的最大置信度
        person_conf = max((d["confidence"] for d in dets if d["class_name"] == "Person"), default=0.0)
        event = {
            "t": time.strftime("%H:%M:%S"),
            "ts": now,
            "type": "NO-Hardhat",
            "cn": PPE_VIOLATION_CN.get("NO-Hardhat", "未戴安全帽"),
            "confidence": round(person_conf, 2),
            "camera": self.cam,
            "duration_s": round(time_without, 1),
        }
        with self.lock:
            self.events.insert(0, event)
            del self.events[50:]
        LINK._log("err", "🚨 安全告警：%s (持续 %.1fs, conf=%.2f)" % (event["cn"], time_without, person_conf))

    # ---- 画框 ----
    def _draw(self, frame, dets):
        if cv2 is None:
            return frame
        for d in dets:
            color = PPE_CLASS_COLORS.get(d["class_id"], (200, 200, 200))
            cv2.rectangle(frame, (d["x1"], d["y1"]), (d["x2"], d["y2"]), color, 2)
            label = "%s %.2f" % (d["class_name"], d["confidence"])
            # 半透明底
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            overlay = frame.copy()
            cv2.rectangle(overlay,
                          (d["x1"], max(0, d["y1"] - th - 8)),
                          (d["x1"] + tw + 6, d["y1"]),
                          color, -1)
            cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
            cv2.putText(frame, label, (d["x1"] + 3, d["y1"] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        # 顶部计数
        persons = sum(1 for d in dets if d["class_name"] == "Person")
        hardhats = sum(1 for d in dets if d["class_name"] == "Hardhat")
        vests = sum(1 for d in dets if d["class_name"] == "Safety Vest")
        no_hardhats = sum(1 for d in dets if d["class_name"] == "NO-Hardhat")
        lines = [
            "Person: %d" % persons,
            "Hardhat: %d" % hardhats,
            "NO-Hardhat: %d" % no_hardhats,
            "Vest: %d" % vests,
        ]
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (8, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        return frame

    # ---- 对外接口 ----
    def start(self, cam="front"):
        if YOLO is None:
            return False, "ultralytics 未安装"
        if cv2 is None:
            return False, "opencv-python 未安装"
        if not os.path.isfile(PPE_MODEL_PATH):
            return False, "模型文件不存在: %s" % PPE_MODEL_PATH
        self.cam = cam
        with self.lock:
            if self.enabled:
                return True, "already running"
            self.enabled = True
            self._stop = False
            self.last_hardhat_time = time.time()
            self.last_alert_time = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        LINK._log("tx", "PPE 检测启动 cam=%s model=%s" % (cam, PPE_MODEL_PATH))
        return True, "started"

    def stop(self):
        with self.lock:
            if not self.enabled:
                return True, "already stopped"
            self.enabled = False
            self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        with self.lock:
            self._annotated = None
            self.last_dets = []
        LINK._log("tx", "PPE 检测停止")
        return True, "stopped"

    def get_annotated_jpeg(self):
        with self.lock:
            return self._annotated

    def status(self):
        now = time.time()
        with self.lock:
            return {
                "enabled": self.enabled,
                "camera": self.cam,
                "model_loaded": self.model is not None,
                "error": self.error,
                "last_frame_age": round(now - self.last_frame_time, 1) if self.last_frame_time else None,
                "last_infer_ms": self.last_infer_ms,
                "det_count": len(self.last_dets),
                "events": list(self.events[:20]),
                "model_path": PPE_MODEL_PATH,
                "conf": PPE_CONF,
                "interval_s": PPE_INTERVAL,
                "persist_s": PPE_PERSIST,
                "cooldown_s": PPE_COOLDOWN,
            }


PPE = None  # 全局单例，main() 里初始化


LINK = None
TOKEN = None                    # --token 启用后，/api/* 与 /video/* 需带 X-API-Token 头
HERE = os.path.dirname(os.path.abspath(__file__))
INDEX_FILE = os.path.join(HERE, "m20_web", "index.html")


# ---------------- OpenAPI 规范（/openapi.json，供集成方生成客户端） ----------------
OPENAPI = {
    "openapi": "3.0.3",
    "info": {
        "title": "Lynx M20 Pro 控制台 API",
        "version": "1.0.0",
        "description":
            "机器人控制台后端。心跳由后端 3Hz 自动维持；行走指令由后端 20Hz 保活下发，"
            "调用方停止上报 0.5s 后自动归零（失联保护）。"
            "启动参数 --token xxx 启用鉴权后，所有 /api/* 与 /video/* 请求需携带 X-API-Token 头。"
            "跨域已放开（Access-Control-Allow-Origin: *）。",
    },
    "servers": [{"url": "http://127.0.0.1:8080", "description": "默认本机；改 --bind 后按实际地址访问"}],
    "paths": {
        "/api/status": {"get": {"tags": ["状态"], "summary": "一次性读取全部状态",
            "description": "适合低频轮询；高频场景请用 /api/stream。",
            "responses": {"200": {"description": "StatusPayload"}}}},
        "/api/stream": {"get": {"tags": ["状态"], "summary": "SSE 实时推送（4Hz）",
            "description": "text/event-stream，每个 data 行为一条 StatusPayload JSON。",
            "responses": {"200": {"description": "事件流"}}}},
        "/api/cmd": {"post": {"tags": ["控制"], "summary": "下发动作/模式/步态指令",
            "requestBody": {"required": True, "content": {"application/json": {
                "schema": {"$ref": "#/components/schemas/CmdRequest"}}}},
            "responses": {"200": {"description": '{"ok": true|false}；false 多为 action 不合法'}}}},
        "/api/velocity": {"post": {"tags": ["控制"], "summary": "设置行走速度（持续生效，20Hz 保活）",
            "description": "写一次持续生效；停止/归零请再调一次全 0 或发 /api/cmd stop。仅常规/辅助模式有效。",
            "requestBody": {"required": True, "content": {"application/json": {
                "schema": {"$ref": "#/components/schemas/VelocityRequest"}}}},
            "responses": {"200": {"description": '{"ok": true}'}}}},
        "/api/config": {"post": {"tags": ["配置"], "summary": "修改机器人 IP",
            "requestBody": {"required": True, "content": {"application/json": {
                "schema": {"type": "object", "required": ["ip"],
                           "properties": {"ip": {"type": "string", "example": "10.21.31.103"}}}}}},
            "responses": {"200": {"description": '{"ok": true, "ip": "..."}'}}}},
        "/video/front": {"get": {"tags": ["视频"], "summary": "前广角 MJPEG 流",
            "description": "multipart/x-mixed-replace，<img src> 或 OpenCV 读入；需机器人 8554 端口可达。",
            "responses": {"200": {"description": "MJPEG 流"},
                          "501": {"description": "服务端未安装 opencv-python"}}}},
        "/video/rear": {"get": {"tags": ["视频"], "summary": "后广角 MJPEG 流",
            "responses": {"200": {"description": "MJPEG 流"}}}},
        "/openapi.json": {"get": {"tags": ["文档"], "summary": "本规范",
            "responses": {"200": {"description": "OpenAPI 3.0 JSON"}}}},
        "/docs": {"get": {"tags": ["文档"], "summary": "接口文档页（人读）",
            "responses": {"200": {"description": "HTML"}}}},
    },
    "components": {"schemas": {
        "CmdRequest": {"type": "object", "required": ["action"],
            "properties": {"action": {"type": "string",
                "enum": ["stand", "lie", "damping", "estop", "stop",
                         "gait_basic", "gait_platform", "gait_stair",
                         "mode_normal", "mode_nav", "mode_aux", "heartbeat",
                         "light_front_on", "light_front_off", "light_back_on", "light_back_off", "light_all_off",
                         "dir_forward", "dir_backward"],
                "description": "stand=站起 lie=趴下 damping=阻尼(旧兼容) estop=软急停 stop=停止行走 "
                               "gait_basic=基础(0x1001) gait_platform=高台(0x1002) gait_stair=楼梯(0x3003) "
                               "mode_*=使用模式 heartbeat=补发心跳 "
                               "light_front_on/off=前灯  light_back_on/off=后灯  light_all_off=全关 "
                               "dir_forward=朝前 dir_backward=朝后(后端 X 轴翻转实现)"}}},
        "VelocityRequest": {"type": "object",
            "properties": {
                "forward": {"type": "number", "minimum": -1, "maximum": 1, "description": "X 前正后负"},
                "side":    {"type": "number", "minimum": -1, "maximum": 1, "description": "Y 左正右负"},
                "yaw":     {"type": "number", "minimum": -1, "maximum": 1, "description": "转向"}}, 
            "example": {"forward": 0.3, "side": 0, "yaw": 0}},
        "StatusPayload": {"type": "object",
            "properties": {
                "ip": {"type": "string"}, "port": {"type": "integer"},
                "online": {"type": "boolean", "description": "2s 内有状态回报"},
                "rx_age": {"type": "number", "nullable": True, "description": "距最后回报秒数"},
                "basic": {"type": "object", "description": "Type=1002 Cmd=6 BasicStatus，含 MotionState/Gait/"
                          "Charge/HES/ControlUsageMode(0常规1导航2辅助)/Direction/OOA(避障 0未启动1空闲2无障碍3避障中)/Sleep 等"},
                "motion": {"type": "object", "description": "Type=1002 Cmd=4 MotionStatus，含 Roll/Pitch/Yaw/"
                           "OmegaZ/LinearX/LinearY/Height/Payload"},
                "cmd_vel": {"type": "object", "description": "当前保活速度 {f,s,y}"},
                "snapshots": {"type": "object", "description": "原始上报，按 Command 码索引"},
                "log": {"type": "array", "items": {"type": "object"}}}},
    }}}


# ---------------- 接口文档页（/docs，人读 + curl 示例） ----------------
DOCS_HTML = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>M20 控制台 API 文档</title>
<style>
 body{margin:0;background:#0f1216;color:#e6edf3;font:14px/1.6 "Segoe UI","Microsoft YaHei",sans-serif;padding:24px}
 h1{font-size:20px} h2{font-size:15px;color:#8b98a5;margin:22px 0 8px;text-transform:uppercase;letter-spacing:.04em}
 code,pre{font-family:Consolas,monospace} code{background:#1d242c;border-radius:4px;padding:1px 6px;color:#9fd0ff}
 pre{background:#12171c;border:1px solid #29323c;border-radius:8px;padding:10px 12px;overflow:auto;color:#cfe6ff}
 table{border-collapse:collapse;width:100%} td,th{border:1px solid #29323c;padding:6px 10px;text-align:left;vertical-align:top}
 th{background:#171c22;color:#8b98a5} td.m{white-space:nowrap;font-family:Consolas,monospace}
 .get{color:#7ee0a0;font-weight:700}.post{color:#7cc4ff;font-weight:700}
 .warn{background:#2a2410;border:1px solid #4a3f16;color:#e8d08a;border-radius:8px;padding:8px 12px}
</style></head><body>
<h1>Lynx M20 Pro 控制台 API</h1>
<p>心跳由后端自动维持（3Hz），集成方无需处理；行走速度写一次持续生效，20Hz 保活下发，停止上报 0.5s 自动归零。
机器可读规范：<code><a style="color:#7cc4ff" href="/openapi.json">/openapi.json</a></code>（可直接导入 Apifox / Postman / 代码生成器）。</p>
<div class="warn">鉴权：服务端以 <code>--token xxx</code> 启用时，以下所有接口需带请求头 <code>X-API-Token: xxx</code>；未启用则免鉴权。</div>

<h2>状态</h2>
<table><tr><th>方法</th><th>路径</th><th>说明</th></tr>
<tr><td class="m"><span class="get">GET</span></td><td class="m">/api/status</td><td>一次性读取全部状态（低频轮询用）</td></tr>
<tr><td class="m"><span class="get">GET</span></td><td class="m">/api/stream</td><td>SSE 实时推送，约 4Hz；<code>data:</code> 行为同 /api/status 的 JSON</td></tr>
</table>
<p>status 返回字段：<code>online</code>（2s 内有回报）、<code>rx_age</code>、<code>basic</code>（MotionState/Gait/Charge/HES/
<b>ControlUsageMode</b>(0常规 1导航 2辅助)/<b>OOA</b>(避障 0未启动 1空闲 2无障碍 3避障中)/Sleep…）、
<code>motion</code>（Roll/Pitch/Yaw/LinearX/LinearY/OmegaZ/Height…）、<code>cmd_vel</code>、<code>snapshots</code>、<code>log</code>。</p>

<h2>控制</h2>
<table><tr><th>方法</th><th>路径</th><th>说明</th></tr>
<tr><td class="m"><span class="post">POST</span></td><td class="m">/api/cmd</td><td>body: <code>{"action": "..."}</code>，
 action ∈ stand站起 / lie趴下 / damping阻尼 / <b>estop软急停</b> / stop停止行走 /
 <b>gait_basic基础(0x1001)</b> / <b>gait_platform高台(0x1002)</b> / <b>gait_stair楼梯(0x3003)</b> /
 mode_normal / mode_nav / mode_aux / heartbeat</td></tr>
<tr><td class="m"><span class="post">POST</span></td><td class="m">/api/velocity</td><td>body: <code>{"forward":-1..1,"side":-1..1,"yaw":-1..1}</code>，
 前正后负、左正右负；仅常规/辅助模式有效</td></tr>
</table>

<h2>视频</h2>
<table><tr><th>方法</th><th>路径</th><th>说明</th></tr>
<tr><td class="m"><span class="get">GET</span></td><td class="m">/video/front</td><td>前广角 MJPEG 流（&lt;img src&gt; 可直接用）</td></tr>
<tr><td class="m"><span class="get">GET</span></td><td class="m">/video/rear</td><td>后广角 MJPEG 流</td></tr>
</table>

<h2>curl 示例</h2>
<pre># 读状态
curl http://127.0.0.1:8080/api/status

# 站立 → 行走 → 停止
curl -X POST http://127.0.0.1:8080/api/cmd  -H "Content-Type: application/json" -d '{"action":"stand"}'
curl -X POST http://127.0.0.1:8080/api/velocity -H "Content-Type: application/json" -d '{"forward":0.3,"side":0,"yaw":0}'
curl -X POST http://127.0.0.1:8080/api/cmd  -H "Content-Type: application/json" -d '{"action":"stop"}'

# 切辅助模式（自动避障）并看避障状态
curl -X POST http://127.0.0.1:8080/api/cmd -d '{"action":"mode_aux"}'
curl http://127.0.0.1:8080/api/status   # 看 basic.OOA

# 开了 --token 时
curl -H "X-API-Token: xxx" http://127.0.0.1:8080/api/status

# 前摄像头画面存成一帧帧 jpg（PowerShell）
curl http://127.0.0.1:8080/video/front -o stream.mjpg</pre>

<h2>Python 集成示例</h2>
<pre>import requests, time
BASE = "http://127.0.0.1:8080"
requests.post(BASE+"/api/cmd", json={"action":"stand"}); time.sleep(5)
requests.post(BASE+"/api/velocity", json={"forward":0.3})
time.sleep(3)
requests.post(BASE+"/api/cmd", json={"action":"stop"})
print(requests.get(BASE+"/api/status").json()["basic"]["ControlUsageMode"])</pre>
</body></html>"""


# ---------------- HTTP 服务 ----------------
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    # ---- 通用响应助手 ----
    def _cors(self):
        """放开跨域：其他系统的 Web 前端也能直接调本服务。"""
        self.send_header("Access-Control-Allow-Origin", "*")

    def _auth(self):
        """--token 启用时校验 X-API-Token；未启用放行。"""
        return not TOKEN or self.headers.get("X-API-Token") == TOKEN

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self._cors()
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _html(self, body):
        b = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self._cors()
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_OPTIONS(self):
        """浏览器跨域预检。"""
        self.send_response(204)
        self._cors()
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, X-API-Token")
        self.send_header("Access-Control-Max-Age", "86400")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/openapi.json":
            self._json(OPENAPI)
        elif p == "/docs":
            self._html(DOCS_HTML)
        elif p in ("/", "/index.html"):
            try:
                with open(INDEX_FILE, "rb") as f:
                    html = f.read()
            except OSError:
                self.send_error(404, "m20_web/index.html 缺失")
                return
            self._html(html)
        elif p.startswith("/api/") or p.startswith("/video/"):
            if not self._auth():
                self._json({"ok": False, "err": "unauthorized: 缺少或错误的 X-API-Token 头"}, 401)
                return
            if p == "/api/status":
                self._json(LINK.payload())
            elif p == "/api/stream":
                self._sse()
            elif p == "/api/ppe/status":
                self._json(PPE.status() if PPE else {"enabled": False, "error": "PPE 未初始化"})
            elif p == "/video/front":
                self._video("front")
            elif p == "/video/rear":
                self._video("rear")
            elif p == "/video/ppe":
                self._video_ppe()
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def _sse(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self._cors()
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()
        try:
            while True:
                self.wfile.write(("data: " + json.dumps(LINK.payload()) + "\n\n").encode("utf-8"))
                self.wfile.flush()
                time.sleep(0.25)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def _video(self, cam):
        """把机器人 RTSP 流转成 MJPEG（multipart/x-mixed-replace），浏览器 <img> 可直接显示。"""
        stream = CAM.get(cam)
        if stream is None:
            self.send_error(404)
            return
        if cv2 is None:
            self.send_error(501, "未安装 opencv-python")
            return
        # 低延迟关键：禁用 FFmpeg 内部缓冲，启用 TCP，禁用帧重排
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;tcp|buffer_size;1024|max_delay;0|"
            "fflags;nobuffer|flags;low_delay|avioflags;direct"
        )
        url = "rtsp://%s:%d/%s" % (LINK.ip, RTSP_PORT, stream)
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self._cors()
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")     # 禁 nginx 类代理缓冲
        self.end_headers()
        cap = None
        try:
            while True:
                if cap is None:                      # 断流自动重连
                    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
                    # OpenCV 缓冲区设到最小（部分版本支持）
                    try:
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                ok, frame = cap.read()
                if not ok:
                    cap.release()
                    cap = None
                    time.sleep(0.3)
                    continue
                h, w = frame.shape[:2]
                if w > 640:                          # 限宽 640，省带宽
                    frame = cv2.resize(frame, (640, int(h * 640 / w)))
                ok2, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if not ok2:
                    continue
                data = buf.tobytes()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(data)).encode() + b"\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()                   # 立即推出去，不等缓冲
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            if cap is not None:
                cap.release()

    def _video_ppe(self):
        """
        读 YOLO 线程已经画好框的 annotated 帧 → JPEG → 全速推流。
        和 YOLO 线程并行：YOLO 负责推理+画框，这里只负责编码推流，互不阻塞。
        框和画面同源对齐（YOLO 在自己 resize+copy 的帧上推理+画框，不漂）。
        """
        if PPE is None or not PPE.enabled:
            self._json({"ok": False, "err": "PPE 未启动"}, 503); return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self._cors()
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        last_ts = 0.0
        try:
            while PPE.enabled:
                with PPE.lock:
                    frame = PPE.last_annotated
                    ts = PPE.last_annotated_time
                if frame is None or ts <= last_ts:
                    time.sleep(0.005); continue
                last_ts = ts
                ok2, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if not ok2: continue
                data = buf.tobytes()
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(data)).encode() + b"\r\n\r\n")
                self.wfile.write(data); self.wfile.write(b"\r\n"); self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_POST(self):
        p = urlparse(self.path).path
        if p.startswith("/api/") and not self._auth():
            self._json({"ok": False, "err": "unauthorized: 缺少或错误的 X-API-Token 头"}, 401)
            return
        n = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(n) if n else b"{}"
        try:
            req = json.loads(raw or b"{}")
        except ValueError:
            req = {}

        if p == "/api/cmd":
            a = req.get("action")
            if a in ("stand", "lie", "damping", "estop"):
                ok = LINK.set_motion(a)
            elif a.startswith("gait_"):
                ok = LINK.set_gait(a.split("_", 1)[1])
            elif a == "mode_normal":
                ok = LINK.set_mode(0)
            elif a == "mode_nav":
                ok = LINK.set_mode(1)
            elif a == "mode_aux":
                ok = LINK.set_mode(2)
            elif a == "heartbeat":
                LINK.heartbeat(); ok = True
            elif a == "stop":
                ok = LINK.set_velocity(0, 0, 0)
            elif a == "light_front_on":
                ok = LINK.set_flashlight(True, None)  # None 表示保持当前后灯状态
            elif a == "light_front_off":
                ok = LINK.set_flashlight(False, None)
            elif a == "light_back_on":
                ok = LINK.set_flashlight(None, True)
            elif a == "light_back_off":
                ok = LINK.set_flashlight(None, False)
            elif a == "light_all_off":
                ok = LINK.set_flashlight(False, False)
            elif a == "dir_forward":
                ok = LINK.set_direction(True)
            elif a == "dir_backward":
                ok = LINK.set_direction(False)
            else:
                ok = False
            self._json({"ok": bool(ok)})
        elif p == "/api/velocity":
            ok = LINK.set_velocity(req.get("forward", 0), req.get("side", 0), req.get("yaw", 0))
            self._json({"ok": bool(ok)})
        elif p == "/api/config":
            ip = (req.get("ip") or "").strip()
            if ip:
                LINK.ip = ip
            self._json({"ok": True, "ip": LINK.ip})
        elif p == "/api/ppe/start":
            if PPE is None:
                self._json({"ok": False, "err": "PPE 未初始化"})
                return
            cam = (req.get("camera") or "front").strip()
            if cam not in CAM:
                self._json({"ok": False, "err": "camera 必须是 front 或 rear"})
                return
            ok, msg = PPE.start(cam)
            self._json({"ok": ok, "msg": msg})
        elif p == "/api/ppe/stop":
            if PPE is None:
                self._json({"ok": False, "err": "PPE 未初始化"})
                return
            ok, msg = PPE.stop()
            self._json({"ok": ok, "msg": msg})
        else:
            self.send_error(404)


def main():
    global LINK, TOKEN, PPE, PPE_MODEL_PATH
    ap = argparse.ArgumentParser(description="Lynx M20 Pro Web 控制台")
    ap.add_argument("--ip", default="10.21.31.103", help="机器人 IP")
    ap.add_argument("--port", type=int, default=30000, help="机器人 UDP 端口")
    ap.add_argument("--http", type=int, default=8080, help="控制台 HTTP 端口")
    ap.add_argument("--bind", default="127.0.0.1",
                    help="HTTP 绑定地址；供其他系统/手机调用时用 0.0.0.0")
    ap.add_argument("--token", default=None,
                    help="接口鉴权令牌；设置后 /api/* 与 /video/* 需带 X-API-Token 头")
    ap.add_argument("--ppe-model", default=PPE_MODEL_PATH,
                    help="PPE 模型 .pt 路径（默认取环境变量 M20_PPE_MODEL 或 D:\\1\\1\\Construction-PPE-Detection-main\\Model\\ppe.pt）")
    args = ap.parse_args()

    TOKEN = args.token
    LINK = M20Link(args.ip, args.port)
    # 允许命令行覆盖模型路径
    if args.ppe_model and args.ppe_model != PPE_MODEL_PATH:
        PPE_MODEL_PATH = args.ppe_model
    PPE = PPEDetector(cam="front")

    print("=" * 56)
    print("  Lynx M20 Pro 控制台")
    print("  机器人 : %s:%d (UDP)" % (args.ip, args.port))
    print("  控制台 : http://%s:%d/" % (args.bind, args.http))
    print("  接口   : http://%s:%d/docs        (OpenAPI: /openapi.json)" % (args.bind, args.http))
    print("  鉴权   : %s" % ("X-API-Token 已启用" if TOKEN else "未启用（--token xxx 可开启）"))
    print("  行走   : 已启用（20Hz 保活 + 0.5s 失联归零）")
    ppe_ok = (YOLO is not None) and (cv2 is not None) and os.path.isfile(PPE_MODEL_PATH)
    print("  PPE    : %s（模型: %s）" % (
        "可用" if ppe_ok else "不可用（缺 ultralytics/cv2 或模型文件）",
        PPE_MODEL_PATH))
    print("=" * 56)

    srv = ThreadingHTTPServer((args.bind, args.http), Handler)
    srv.daemon_threads = True
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[退出] 控制台已停止（心跳随之停止）")
        LINK._stop = True
        sys.exit(0)


if __name__ == "__main__":
    main()
