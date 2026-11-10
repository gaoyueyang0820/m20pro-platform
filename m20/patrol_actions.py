"""People counts and optional FastBee device readings at a stopped waypoint."""
import json
import os
from pathlib import Path
import re
import time
from contextlib import suppress
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit, urlunsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


def station_spec(value):
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('航点任务格式无效')
    camera = value.get('camera', 'front')
    pan = value.get('pan', 'none')
    device = str(value.get('iot_device_id') or '').strip()
    if camera not in ('front', 'rear', 'ball') or pan not in ('none', 'left', 'right'):
        raise ValueError('航点相机或云台方向无效')
    if pan != 'none' and camera != 'ball':
        raise ValueError('转动云台需选择外接球机')
    if device and not re.fullmatch(r'[0-9]{1,20}', device):
        raise ValueError('请输入物联网平台的设备 ID')
    return dict(camera=camera, pan=pan, iot_device_id=device)


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class FastBee:
    """Scoped client for the documented API and the upstream SIP v2 controller."""
    def __init__(self, config):
        self.base = str(config.get('iot_url', '')).rstrip('/')
        self.user = config.get('iot_user', '')
        self.password = config.get('iot_password', '')
        self.token = config.get('iot_token', '')
        self.device = str(config.get('sip_device', ''))
        self.channel = str(config.get('sip_channel', ''))
        self.ptz_enabled = config.get('fastbee_ptz') is True
        self.opener = build_opener(ProxyHandler({}), NoRedirect())
        parts = urlsplit(self.base)
        self.configured = bool(parts.scheme in ('http', 'https') and parts.hostname
                               and not (parts.username or parts.password or parts.query or parts.fragment)
                               and (self.token or self.user and self.password))

    def _json(self, path, method='GET', data=None, token=None, timeout=3):
        headers = {'Accept': 'application/json'}
        if token:
            headers['Authorization'] = 'Bearer ' + token
        if data is not None:
            headers['Content-Type'] = 'application/json'
        req = Request(self.base+path, json.dumps(data).encode() if data is not None else None,
                      headers=headers, method=method)
        try:
            with self.opener.open(req, timeout=timeout) as response:
                status, raw = response.status, response.read(1_000_000)
        except HTTPError as error:
            return error.code, {}
        except (OSError, URLError):
            raise ValueError('物联网平台连接超时或不可达') from None
        try:
            payload = json.loads(raw)
        except (ValueError, UnicodeError):
            raise ValueError('物联网平台返回的内容不是 JSON') from None
        if not isinstance(payload, dict):
            raise ValueError('物联网平台返回格式无效')
        return status, payload

    def _login(self):
        if not self.configured or not (self.user and self.password):
            raise ValueError('请配置有效的物联网平台凭据')
        status, challenge = self._json('/captchaImage?showCode=true')
        if status != 200:
            raise ValueError('物联网登录服务不可用')
        body = dict(username=self.user, password=self.password)
        if challenge.get('captchaEnabled') is not False:
            answer = challenge.get('resultCode')
            if answer is None or not challenge.get('uuid'):
                raise ValueError('物联网平台需要图形验证码，请配置有效的登录令牌')
            body.update(code=str(answer), uuid=challenge['uuid'])
        status, payload = self._json('/login', 'POST', body)
        inner = payload.get('data')
        token = payload.get('token') or (inner.get('token') if isinstance(inner, dict) else None)
        if status != 200 or payload.get('code') != 200 or not isinstance(token, str) or not token:
            raise ValueError('物联网平台认证失败')
        self.token = token

    def _call(self, path, method='GET', timeout=3):
        if not self.configured:
            raise ValueError('物联网平台尚未配置')
        if not self.token:
            self._login()
        status, payload = self._json(path, method, token=self.token, timeout=timeout)
        if status == 401 or payload.get('code') == 401:
            self._login()
            status, payload = self._json(path, method, token=self.token, timeout=timeout)
        if status != 200 or payload.get('code') != 200:
            code = payload.get('code')
            if not isinstance(code, int):
                code = status
            raise ValueError('物联网平台未接受请求（%s）' % code)
        return payload.get('data')

    @property
    def sip_ready(self):
        return self.configured and all(re.fullmatch(r'[0-9]{20}', value)
                                       for value in (self.device, self.channel))

    def play(self):
        if not self.sip_ready:
            raise ValueError('外接球机未配置 RTSP 地址或国标通道')
        data = self._call(f'/sip/player/play/{self.device}/{self.channel}', timeout=8)
        candidates = []
        def collect(value):
            if isinstance(value, str):
                parts = urlsplit(value)
                if parts.scheme in ('rtsp', 'http', 'https') and parts.hostname:
                    path = parts.path
                    if path.endswith('.live.mp4'):
                        path = path[:-4] + '.flv'
                    if parts.scheme == 'rtsp' or path.endswith('.live.flv'):
                        candidates.append(urlunsplit((parts.scheme, parts.netloc, path, parts.query, '')))
            elif isinstance(value, dict):
                for row in value.values():
                    collect(row)
        collect(data)
        if not candidates:
            raise ValueError('国标球机暂未返回可解码的实时视频地址')
        return min(candidates, key=lambda url: not url.startswith('rtsp://'))

    def ptz(self, direction):
        if not self.ptz_enabled or not self.sip_ready:
            raise ValueError('外接云台控制尚未配置')
        suffix = 'STOP' if direction == 'stop' else direction.upper() + '/35'
        result = self._call(f'/sip/ptz/ptz/{self.device}/{self.channel}/{suffix}', 'POST', timeout=1.5)
        if result is not True:
            raise ValueError('国标云台通道未接受控制')

    def telemetry(self, device):
        data = self._call('/iot/device/runningStatus?' + urlencode(dict(deviceId=device)))
        if not isinstance(data, dict):
            return {}
        values = data.get('thingsModelValue', data)
        if isinstance(values, str):
            try:
                values = json.loads(values)
            except ValueError:
                return {}
        if isinstance(values, list):
            values = {str(row.get('name') or row.get('id', '')): row.get('value') if row.get('value') is not None else row.get('shadow')
                      for row in values if isinstance(row, dict)}
        if not isinstance(values, dict):
            return {}
        # Return measurements only, never the underlying device object.
        return {str(key)[:64]: value for key, value in list(values.items())[:40]
                if type(value) in (int, float) or isinstance(value, str)
                and re.fullmatch(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', value.strip())}


class StationActions:
    def __init__(self, aos, config=None):
        self.aos = aos
        self.config = config or {}
        self.ball_rtsp = self.config.get('ball_rtsp', '')
        self.fastbee = FastBee(self.config)
        self.model = None
        self.person_classes = None

    def _people_model_path(self):
        from m20.vision_models import default_path
        if self.config.get('people_model'):
            return Path(self.config['people_model'])
        weights = default_path('counting')
        exported = weights.with_suffix('.onnx')
        return exported if exported.is_file() else weights

    def capabilities(self):
        from importlib.util import find_spec
        model = self._people_model_path()
        runtime = 'onnxruntime' if model.suffix=='.onnx' else 'ultralytics'
        ready = model.is_file() and find_spec('cv2') is not None and find_spec(runtime) is not None
        return dict(people_count=ready, ball_camera=bool(self.ball_rtsp or self.fastbee.sip_ready),
                    ptz=self.fastbee.sip_ready and self.fastbee.ptz_enabled, iot=self.fastbee.configured)

    def validate(self, points):
        actions = [p['station_action'] for p in points if p.get('station_action')]
        if not actions:
            return
        caps = self.capabilities()
        if not caps['people_count']:
            raise ValueError('GOS 人数识别环境或本地模型尚未配置')
        if any(a['camera'] == 'ball' for a in actions) and not caps['ball_camera']:
            raise ValueError('外接球机尚未配置')
        if any(a['pan'] != 'none' for a in actions) and not caps['ptz']:
            raise ValueError('外接云台控制尚未配置')

    @staticmethod
    def _wait(seconds, guard):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            guard()
            time.sleep(min(.05, max(0., end-time.monotonic())))

    def _pan(self, direction, guard):
        guard()
        # A network timeout may occur after the camera received the move.
        try:
            self.fastbee.ptz(direction)
            self._wait(.65, guard)
        finally:
            for _ in range(2):
                try:
                    self.fastbee.ptz('stop')
                    break
                except ValueError:
                    continue
            else:
                raise ValueError('云台停止未确认，巡检已停在当前航点')
        self._wait(1.2, guard)

    def _stream(self, camera):
        if camera == 'ball':
            return self.ball_rtsp or self.fastbee.play()
        return 'rtsp://%s:8554/video%d' % (self.aos, 1 if camera == 'front' else 2)

    def load_people_model(self, worker=False):
        if self.model is not None:
            return
        model_path = self._people_model_path()
        if not model_path.is_file():
            raise ValueError('GOS 本地人数识别模型不可用')
        if model_path.suffix=='.onnx':
            from m20.onnx_people import OnnxPeople
            self.model = OnnxPeople(model_path)
            return
        if worker:
            import torch
            torch.set_num_threads(1)
            with suppress(RuntimeError):
                torch.set_num_interop_threads(1)
        os.environ.setdefault('YOLO_OFFLINE', 'true')
        os.environ.setdefault('YOLO_AUTOINSTALL', 'false')
        from ultralytics import YOLO
        model = YOLO(str(model_path))
        labels = model.names
        classes = [i for i, name in (labels.items() if isinstance(labels, dict) else enumerate(labels))
                   if str(name).casefold() == 'person']
        if not classes:
            raise ValueError('当前模型缺少 person 类别')
        self.model, self.person_classes = model, classes

    def count_people(self, frame, size=480):
        from m20.vision_models import detections
        self.load_people_model()
        if self.person_classes is None:
            return int(self.model.count(frame))
        return len(detections(self.model, frame, confidence=.3, size=size, classes=self.person_classes))

    def open_camera(self, camera, guard=lambda: None):
        import cv2
        from m20.camera_streams import capture_params
        from m20.ffmpeg_capture import FFmpegCapture, ffmpeg_major, find_ffmpeg
        guard()
        binary = find_ffmpeg()
        legacy_version = None
        if not binary and os.name=='posix':
            binary = find_ffmpeg(4)
            if binary:
                legacy_version = ffmpeg_major(binary)
        def open_stream(source):
            if urlsplit(source).scheme not in ('rtsp', 'http', 'https'):
                raise ValueError('巡检相机需要 RTSP 或 HTTP 实时视频')
            try:
                return (FFmpegCapture(cv2, source, binary, open_timeout=4., legacy_version=legacy_version) if binary
                        else cv2.VideoCapture(source, cv2.CAP_FFMPEG, capture_params(cv2)))
            except (OSError, cv2.error):
                raise ValueError('巡检相机解码服务不可用') from None
        capture = open_stream(self._stream(camera))
        if not capture.isOpened() and camera == 'ball' and self.ball_rtsp and self.fastbee.sip_ready:
            capture.release()
            guard()
            capture = open_stream(self.fastbee.play())
        if not capture.isOpened():
            capture.release()
            raise ValueError('无法取得巡检相机实时视频')
        return capture

    def _counts(self, camera, guard):
        self.load_people_model()
        capture = self.open_camera(camera, guard)
        try:
            counts = []
            for _ in range(3):
                guard()
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise ValueError('航点相机画面中断')
                counts.append(self.count_people(frame))
            guard()
            return counts
        finally:
            capture.release()

    def run(self, action, guard):
        if action['pan'] != 'none':
            self._pan(action['pan'], guard)
        counts = self._counts(action['camera'], guard)
        result = dict(camera=action['camera'], pan=action['pan'], samples=counts, people=max(counts),
                      counted_at=time.time())
        if action['iot_device_id']:
            guard()
            device = action['iot_device_id']
            result['iot_device_id'] = device
            try:
                result['measurements'] = self.fastbee.telemetry(device)
                if not result['measurements']:
                    result['iot_error'] = '设备暂未上报当前读数'
            except ValueError as error:
                result['iot_error'] = str(error)
        guard()
        return result
