"""People counts and optional FastBee device readings at a stopped waypoint."""
import json
import math
import os
from pathlib import Path
import re
import time
from contextlib import suppress
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit, urlunsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener


def device_id(value):
    if not re.fullmatch(r'[0-9]{1,20}', str(value).strip()):
        raise ValueError('请选择有效的物联网设备')
    return str(value).strip()


def device_ids(value):
    if not isinstance(value, list) or len(value) > 8:
        raise ValueError('每项任务最多选择 8 台物联网设备')
    return list(dict.fromkeys(device_id(item) for item in value))


def station_spec(value):
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError('航点任务格式无效')
    camera = value.get('camera', 'front')
    pan = value.get('pan', 'none')
    device = str(value.get('iot_device_id') or '').strip()
    devices = device_ids(value.get('iot_device_ids', []))
    if camera not in ('none', 'front', 'rear', 'ball') or pan not in ('none', 'left', 'right'):
        raise ValueError('航点相机或云台方向无效')
    if pan != 'none' and camera != 'ball':
        raise ValueError('转动云台需选择外接球机')
    if device:
        device = device_id(device)
    if camera == 'none' and not (devices or device):
        return None
    result = dict(camera=camera, pan=pan, iot_device_id='' if devices else device)
    if devices:
        result['iot_device_ids'] = devices
    return result


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


class IoTLoginChallenge(ValueError):
    def __init__(self, uuid, image):
        super().__init__('物联网登录需要验证码，请在连接中心验证')
        self.challenge = dict(uuid=uuid, image=image)


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

    def _login(self, code=None, uuid=None):
        if not self.configured or not (self.user and self.password):
            raise ValueError('请配置有效的物联网平台凭据')
        body = dict(username=self.user, password=self.password)
        if code and uuid:
            body.update(code=code, uuid=uuid)
        else:
            status, challenge = self._json('/captchaImage?showCode=true')
            if status != 200:
                raise ValueError('物联网登录服务不可用')
            if challenge.get('captchaEnabled') is not False:
                answer, key, image = (challenge.get(name) for name in ('resultCode', 'uuid', 'img'))
                if answer is not None and key:
                    body.update(code=str(answer), uuid=key)
                elif (isinstance(key, str) and re.fullmatch(r'[A-Za-z0-9_-]{1,96}', key)
                      and isinstance(image, str) and len(image) <= 256_000
                      and re.fullmatch(r'[A-Za-z0-9+/=]+', image)):
                    raise IoTLoginChallenge(key, image)
                else:
                    raise ValueError('无法读取物联网验证码，请使用已授权的登录令牌')
        status, payload = self._json('/login', 'POST', body)
        inner = payload.get('data')
        token = payload.get('token') or (inner.get('token') if isinstance(inner, dict) else None)
        if status != 200 or payload.get('code') != 200 or not isinstance(token, str) or not token:
            raise ValueError('物联网平台认证失败')
        self.token = token

    def _payload(self, path, method='GET', timeout=3):
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
        return payload

    def _call(self, path, method='GET', timeout=3):
        return self._payload(path, method, timeout).get('data')

    def devices(self, page=1, name=''):
        size = 20
        params = dict(pageNum=page, pageSize=size)
        if name:
            params['deviceName'] = name
        payload = self._payload('/iot/device/list?' + urlencode(params))
        data = payload.get('data')
        listing = data if isinstance(data, dict) and 'rows' in data else payload
        rows = listing.get('rows')
        if not isinstance(rows, list):
            raise ValueError('物联网平台未返回设备列表')
        devices = []
        for row in rows[:size]:
            if not isinstance(row, dict):
                continue
            try:
                key = device_id(row.get('deviceId', row.get('id')))
            except ValueError:
                continue
            status = str(row.get('status', '')).casefold()
            state = {'3': 'online', '4': 'offline', 'online': 'online', 'offline': 'offline'}.get(status, 'unknown')
            devices.append(dict(id=key, name=str(row.get('deviceName') or row.get('name') or key)[:120], status=state))
        total = listing.get('total')
        more = page*size < int(total) if str(total).isdigit() else len(rows) >= size
        return dict(devices=devices, page=page, has_more=more)

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
        data = self._call('/iot/device/runningStatus?' + urlencode(dict(deviceId=device_id(device))))
        values = data.get('thingsModelValue', data) if isinstance(data, dict) else data
        readings = self._measurements(values, model=isinstance(data, list) or isinstance(data, dict) and 'thingsModelValue' in data)
        serial = data.get('serialNumber') if isinstance(data, dict) else None
        if not readings and serial and len(str(serial)) <= 128:
            runtime = self._call('/iot/runtime/running?' + urlencode(dict(serialNumber=serial)))
            readings = self._measurements(runtime, model=True)
        return readings

    @staticmethod
    def _measurements(values, model):
        if isinstance(values, str):
            try:
                values = json.loads(values)
            except ValueError:
                return {}
        if isinstance(values, list):
            fields = {}
            for row in values:
                if not isinstance(row, dict):
                    continue
                name = row.get('name_zh_CN') or row.get('name') or row.get('id')
                datatype = row.get('datatype')
                unit = datatype.get('unit') if isinstance(datatype, dict) else None
                if name:
                    label = '%s (%s)' % (name, unit) if unit else str(name)
                    fields[label] = row.get('value') if row.get('value') is not None else row.get('shadow')
            values = fields
            model = True
        if not isinstance(values, dict):
            return {}
        result = {}
        for key, value in values.items():
            if key in ('deviceId', 'productId', 'serialNumber', 'status') and not model:
                continue
            if type(value) in (int, bool) or type(value) is float and math.isfinite(value):
                result[str(key)[:64]] = value
            elif isinstance(value, str) and (model or re.fullmatch(r'[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?', value.strip())):
                result[str(key)[:64]] = value[:64]
            if len(result) >= 40:
                break
        return result

    def read_devices(self, devices, guard=lambda: None):
        rows = []
        for device in devices:
            guard()
            row = dict(device_id=device)
            try:
                row['measurements'] = self.telemetry(device)
                if not row['measurements']:
                    row['error'] = '设备暂未上报读数'
            except ValueError as error:
                row['error'] = str(error)
            rows.append(row)
        return rows


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
        if any(a['camera'] != 'none' for a in actions) and not caps['people_count']:
            raise ValueError('GOS 人数识别环境或本地模型尚未配置')
        if any(a['camera'] == 'ball' for a in actions) and not caps['ball_camera']:
            raise ValueError('外接球机尚未配置')
        if any(a['pan'] != 'none' for a in actions) and not caps['ptz']:
            raise ValueError('外接云台控制尚未配置')
        if any(a.get('iot_device_ids') for a in actions) and not caps['iot']:
            raise ValueError('请先在连接中心接入物联网平台')

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
        result = dict(sampled_at=time.time())
        if action['camera'] != 'none':
            counts = self._counts(action['camera'], guard)
            result.update(camera=action['camera'], pan=action['pan'], samples=counts, people=max(counts),
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
        if action.get('iot_device_ids'):
            result['devices'] = self.fastbee.read_devices(action['iot_device_ids'], guard)
        guard()
        return result
