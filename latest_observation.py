"""Optional latest-only observation worker; no built-in robot or network I/O.

The supplied fetch(context, stop_event) must use the captured context and a
bounded timeout. Control reads never wait for fetch. Every packet must already
carry scan_now's original monotonic _valid_until; reading/repeating never renews
it. The caller owns actuator arbitration and must recheck its run/context.
"""
import copy
import math
import threading
import time


class TransientObservationError(Exception):
    """Only a transport failure; existing cache may live until its SAME deadline."""


class ObservationContextChanged(Exception):
    """The captured connection or collection session no longer owns this data."""


class LatestObservationPump:
    def __init__(self,fetch,*,clock=time.monotonic,poll_interval=.02,error_interval=.04):
        self.fetch=fetch;self.clock=clock;self.poll_interval=max(.001,float(poll_interval));self.error_interval=max(.001,float(error_interval))
        self.lock=threading.RLock();self.generation=0;self.running=False;self.thread=None;self.stop_event=None
        self.latest=None;self.sequence=0;self.last_stamp=None;self.last_error=None;self.error_kind=None
        self.counts=dict(received=0,published=0,duplicates=0,reordered=0,expired=0,rejected=0,late_results=0)

    def start(self,context=None):
        """Start a fresh connection generation; old in-flight results are ignored.

        context must be a deepcopy-able description, not a mutable app pointer
        or a live connection. fetch owns its actual bounded I/O implementation.
        """
        captured=copy.deepcopy(context)
        with self.lock:
            if self.stop_event is not None:self.stop_event.set()
            self.generation+=1;generation=self.generation;stop=threading.Event();self.stop_event=stop
            self.running=True;self.latest=None;self.sequence=0;self.last_stamp=None;self.last_error=None;self.error_kind=None
            thread=threading.Thread(target=self._run,args=(generation,captured,stop,self.fetch),daemon=True,name='m20-observation-'+str(generation))
            self.thread=thread
        try:thread.start()
        except Exception:
            with self.lock:
                if generation==self.generation:
                    self.running=False;stop.set();self.last_error='observation worker did not start';self.error_kind='invalidated'
            raise
        return generation

    def cancel(self):
        """Invalidate synchronously; never wait for a possibly pending RPC."""
        with self.lock:
            self.running=False;self.latest=None
            if self.stop_event is not None:self.stop_event.set()

    def join(self,timeout=.1):
        thread=self.thread
        if thread is not None:thread.join(timeout=max(0.,float(timeout)))
        return thread is None or not thread.is_alive()

    def _current(self,generation):
        return self.running and generation==self.generation and not self.stop_event.is_set()

    def _publish(self,packet,generation):
        received=self.clock()
        with self.lock:
            if not self._current(generation):self.counts['late_results']+=1;return False
            self.counts['received']+=1
            if not isinstance(packet,dict):return self._reject('observation is not an object')
            stamp=packet.get('stamp');deadline=packet.get('_valid_until')
            if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in [stamp,deadline]):
                return self._reject('observation lacks finite source stamp or original deadline')
            if deadline>received+.65+1e-6:return self._reject('observation deadline exceeds the 650 ms source budget')
            if deadline<=received:
                self.counts['expired']+=1;self.last_error='received observation already expired';self.error_kind='temporal';return False
            if self.last_stamp is not None and stamp<self.last_stamp:
                self.counts['reordered']+=1;return False
            if self.last_stamp is not None and stamp==self.last_stamp:
                self.counts['duplicates']+=1
                # A duplicate cannot change geometry or buy another lease. A
                # more conservative deadline can shorten the existing one.
                if self.latest is not None:
                    self.latest['_valid_until']=min(self.latest['_valid_until'],deadline)
                return False
            self.latest=copy.deepcopy(packet);self.latest['_pump_received_monotonic']=received
            self.sequence+=1;self.last_stamp=stamp;self.counts['published']+=1;self.last_error=None;self.error_kind=None
            return True

    def _reject(self,message):
        self.counts['rejected']+=1;self.latest=None;self.last_error=message;self.error_kind='invalidated';return False

    def _error(self,exc,generation):
        with self.lock:
            if not self._current(generation):return
            self.last_error=str(exc);self.error_kind=('context_changed' if isinstance(exc,ObservationContextChanged)
                                                    else 'transport' if isinstance(exc,TransientObservationError) else 'invalidated')
            # Unknown/geometry/pose errors must not silently reuse an old scene.
            # Adapters may explicitly classify a transport-only error if useful.
            if self.error_kind!='transport':self.latest=None

    def _run(self,generation,context,stop,fetch):
        while not stop.is_set():
            with self.lock:
                if not self._current(generation):return
            delay=self.poll_interval
            try:packet=fetch(context,stop)
            except Exception as exc:self._error(exc,generation);delay=self.error_interval
            else:self._publish(packet,generation)
            if stop.wait(delay):return

    def read(self,generation=None,after_sequence=None):
        """Nonblocking snapshot; caller checks run/context again before driving.

        state: waiting / fresh / cached / expired / invalidated / cancelled /
        generation_changed. A cached observation can justify a recomputation
        only until its original deadline, never a new command validity period.
        """
        with self.lock:
            now=self.clock();result=dict(generation=self.generation,sequence=self.sequence,observation=None,
                                         error=self.last_error,error_kind=self.error_kind,remaining_s=0.)
            if generation is not None and generation!=self.generation:return dict(result,state='generation_changed')
            if not self.running:return dict(result,state='cancelled')
            if self.latest is None:return dict(result,state=self.error_kind if self.error_kind in ['invalidated','context_changed'] else 'waiting')
            remaining=self.latest['_valid_until']-now
            if remaining<=0:return dict(result,state='expired')
            packet=copy.deepcopy(self.latest)
            # Age derives from the unchanged 650 ms deadline. Never replace
            # _valid_until with now + a fresh duration, even on repeated reads.
            packet['age']=max(0.,.65-remaining)
            packet['_pump_generation']=self.generation;packet['_pump_sequence']=self.sequence
            packet['_cache_wait_ms']=max(0.,now-packet['_pump_received_monotonic'])*1000
            state='fresh' if after_sequence is None or self.sequence>after_sequence else 'cached'
            return dict(result,state=state,remaining_s=remaining,observation=packet)

    def status(self):
        with self.lock:
            return dict(running=self.running,generation=self.generation,sequence=self.sequence,last_stamp=self.last_stamp,
                        error=self.last_error,error_kind=self.error_kind,worker_alive=bool(self.thread and self.thread.is_alive()),
                        counters=dict(self.counts))
