"""Route task collection contracts. Samplers return measurements, not controls."""
from dataclasses import dataclass
import re
import time


class PeopleSampler:
    def __init__(self, source, spec):
        import cv2
        from m20.camera_streams import CameraFeed

        cv2.setNumThreads(1)
        source.load_people_model(worker=True)
        self.source = source
        self.sequence = -1
        camera = spec['camera']
        self.feed = CameraFeed(cv2, source.aos, camera,
                               open_capture=lambda: source.open_camera(camera), encode=False)
        self.feed.subscribe()

    def sample(self):
        packet = self.feed.read_raw(self.sequence, 3.)
        if packet is None:
            raise ValueError('暂未取得新鲜相机画面')
        self.sequence, frame, captured_at, _ = packet
        return dict(sampled_at=captured_at, values={'people': self.source.count_people(frame, size=416)})

    def close(self):
        self.feed.close()


class IoTSampler:
    def __init__(self, source, spec):
        self.fastbee = source.fastbee
        self.device = spec['device_id']

    def sample(self):
        readings = self.fastbee.telemetry(self.device)
        if not readings:
            raise ValueError('设备暂未上报读数')
        return dict(sampled_at=time.time(), values=readings)

    def close(self):
        pass


@dataclass(frozen=True)
class Collector:
    title: str
    default_interval: int
    min_interval: int
    fields: tuple
    factory: object
    available: object


COLLECTORS = {
    'people_count': Collector(
        '人数统计', 2, 1,
        ({'key': 'camera', 'label': '相机', 'choices': {'front': '前视', 'rear': '后视', 'ball': '外接球机'}},),
        PeopleSampler, lambda caps, spec: caps['people_count'] and (spec['camera'] != 'ball' or caps['ball_camera'])),
    'iot_readings': Collector(
        '物联网读数', 5, 2,
        ({'key': 'device_id', 'label': '设备 ID', 'pattern': r'[0-9]{1,20}'},),
        IoTSampler, lambda caps, spec: caps['iot']),
}


def collector_catalog():
    return [dict(kind=kind, title=entry.title, interval_s=entry.default_interval,
                 min_interval_s=entry.min_interval, max_interval_s=300, fields=list(entry.fields))
            for kind, entry in COLLECTORS.items()]


def collector_specs(items):
    if items is None:
        return []
    if not isinstance(items, list) or len(items) > len(COLLECTORS):
        raise ValueError('沿途采集配置无效')
    specs = []
    kinds = set()
    for item in items:
        kind = item.get('kind') if isinstance(item, dict) else None
        if not isinstance(kind, str) or kind not in COLLECTORS or kind in kinds:
            raise ValueError('请选择不同的已支持采集项')
        entry = COLLECTORS[kind]
        interval = item.get('interval_s', entry.default_interval)
        if type(interval) is not int or not entry.min_interval <= interval <= 300:
            raise ValueError('%s采集间隔应为 %d 至 300 秒' % (entry.title, entry.min_interval))
        spec = dict(kind=kind, interval_s=interval)
        for field in entry.fields:
            key = field['key']
            choices = field.get('choices')
            if choices is not None:
                value = item.get(key, next(iter(choices)))
                valid = isinstance(value, str) and value in choices
            else:
                value = str(item.get(key, '')).strip()
                valid = bool(re.fullmatch(field['pattern'], value))
            if not valid:
                raise ValueError('请核对%s的%s' % (entry.title, field['label']))
            spec[key] = value
        kinds.add(kind)
        specs.append(spec)
    return specs


def check_collectors(specs, capabilities):
    for spec in specs:
        entry = COLLECTORS[spec['kind']]
        if not entry.available(capabilities, spec):
            raise ValueError('GOS %s尚未就绪，请核对相机、识别环境或物联网连接' % entry.title)
