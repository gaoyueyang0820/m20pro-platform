"""Unprivileged host agent delegates only read-only sensor requests over loopback."""
import json
import os
from urllib.request import Request,urlopen
from urllib.error import HTTPError,URLError
from observation_errors import sync_error


class SensorClient:
    def call(self,path,data=None):
        headers={'Content-Type':'application/json','X-API-Token':os.environ.get('M20_BRIDGE_TOKEN','')}
        request=Request('http://127.0.0.1:8094'+path,data=json.dumps(data).encode() if data is not None else None,headers=headers)
        try:
            with urlopen(request,timeout=2) as response:result=json.load(response)
        except HTTPError as exc:
            try:payload=json.load(exc);message=payload.get('err','感知服务返回错误')
            except Exception:payload=None;message=str(exc)
            pending=sync_error(payload,path,data)
            if pending:raise pending
            raise ValueError(message)
        except (URLError,OSError):raise ValueError('只读感知服务尚未就绪，请更新 NOS/GOS 平台服务')
        if not result.get('ok'):
            pending=sync_error(result,path,data)
            if pending:raise pending
            raise ValueError(result.get('err','感知服务返回错误'))
        return result

    def status(self):return self.call('/sensors/status')
    def preview(self,topic):return self.call('/sensors/cloud',dict(topic=topic))
    def autonomy(self,mapping=False):return self.call('/sensors/autonomy',dict(mapping=mapping))
    def observation(self):return self.call('/sensors/observation')
    def close(self):pass
