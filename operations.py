"""Bounded asynchronous work journal; never stores request secrets."""
import copy
import threading
import time
import uuid


class Operations:
    def __init__(self):
        self.lock = threading.RLock()
        self.jobs = {}

    def submit(self, kind, work):
        with self.lock:
            if sum(j['status'] == 'running' for j in self.jobs.values()) >= 4:
                raise ValueError('后台已有 4 个任务，请等待完成')
            key = uuid.uuid4().hex
            job = dict(id=key, kind=kind, status='running', progress=0, message='准备中',
                       started=time.time(), result=None, error=None)
            self.jobs[key] = job
            old = [k for k,v in self.jobs.items() if v['status'] != 'running']
            for k in old[:-80]: self.jobs.pop(k, None)
        def update(progress, message):
            with self.lock:
                job.update(progress=max(0,min(1,float(progress))), message=str(message)[:600])
        def run():
            try:
                result = work(update)
                with self.lock: job.update(status='completed', progress=1, message='已取消，收尾已完成' if isinstance(result,dict) and result.get('cancelled') else '已完成', result=result)
            except Exception as exc:
                with self.lock: job.update(status='failed', error=str(exc)[:1200], message='操作失败')
            finally:
                with self.lock: job['ended'] = time.time()
        try:threading.Thread(target=run, daemon=True).start()
        except Exception as exc:
            with self.lock:job.update(status='failed',error='后台线程启动失败：'+str(exc)[:1000],message='任务未能启动',ended=time.time())
            raise ValueError('后台任务未能启动，请稍后重试') from exc
        return dict(ok=True, job_id=key)

    def get(self, key=None):
        with self.lock:
            if key:
                if key not in self.jobs: raise ValueError('后台任务不存在')
                return copy.deepcopy(self.jobs[key])
            return sorted(copy.deepcopy(list(self.jobs.values())), key=lambda j:j['started'], reverse=True)
