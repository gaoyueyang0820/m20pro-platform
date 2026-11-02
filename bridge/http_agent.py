"""Small localhost-only agent server, shared by the three optional adapters."""
import hmac
import json
import os
import signal
import sys
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from observation_errors import error_payload


def serve(port, dispatch, bind='127.0.0.1'):
    token=os.environ.get('M20_BRIDGE_TOKEN','')
    if bind not in ['127.0.0.1','localhost'] and not token:
        raise ValueError('非本机代理必须配置 M20_BRIDGE_TOKEN')
    class Handler(BaseHTTPRequestHandler):
        # Headers and JSON are separate writes. Across the SSH/Wi-Fi chain,
        # Nagle plus delayed ACK can add a delay to each small sensor response.
        disable_nagle_algorithm = True
        def log_message(self,*args): pass
        def reply(self,obj,status=200):
            raw=json.dumps(obj,ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        def run_request(self,post=False):
            req=None
            try:
                if self.headers.get('Origin'):
                    return self.reply(dict(ok=False,err='代理仅接收服务端请求'),403)
                if token and not hmac.compare_digest(self.headers.get('X-API-Token',''),token):
                    return self.reply(dict(ok=False,err='代理令牌不正确'),401)
                req=None
                if post:
                    if not self.headers.get('Content-Type','').startswith('application/json'):
                        return self.reply(dict(ok=False,err='需要 application/json'),415)
                    size=int(self.headers.get('Content-Length',0))
                    if not 0<=size<=4_000_000: raise ValueError('请求过大')
                    req=json.loads(self.rfile.read(size) or b'{}')
                    if not isinstance(req,dict): raise ValueError('请求必须是 JSON 对象')
                self.reply(dispatch(urlparse(self.path).path,req))
            except (ValueError,KeyError,TypeError) as exc:
                self.reply(error_payload(exc,urlparse(self.path).path,req),400)
            except (BrokenPipeError,ConnectionResetError): pass
            except Exception as exc:
                self.reply(dict(ok=False,err='代理执行失败: '+str(exc)),500)
        def do_GET(self): self.run_request()
        def do_POST(self): self.run_request(True)
    server=ThreadingHTTPServer((bind,port),Handler)
    if threading_is_main():
        def terminate(signum,frame):raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,terminate)
    server.daemon_threads=True
    print('M20 代理监听 http://%s:%d （请通过 SSH 隧道访问）'%(bind,port))
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()


def threading_is_main():
    import threading
    return threading.current_thread() is threading.main_thread()
