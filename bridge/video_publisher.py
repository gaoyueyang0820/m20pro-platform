#!/usr/bin/env python3
"""Run the existing GST pipeline with the upstream locked-bin startup fix.

Backports the behavior of gst-rtsp-server ed189750d55111cbcf1e6f8a80dbf573adc4b16a
without replacing vendor GStreamer/MPP libraries. No frame callbacks or copies.
"""
import json
import signal
import sys
import time


def main(argv):
    import gi
    gi.require_version('Gst','1.0')
    from gi.repository import Gst
    Gst.init(None)
    started=time.monotonic()
    def report(event,**fields):
        print(json.dumps(dict(event=event,seconds=round(time.monotonic()-started,3),**fields)),flush=True)
    pipeline=Gst.parse_launchv(argv)
    iterator=pipeline.iterate_recurse();fixed=0
    while True:
        result,element=iterator.next()
        if result==Gst.IteratorResult.RESYNC:iterator.resync();continue
        if result!=Gst.IteratorResult.OK:break
        factory=element.get_factory()
        if factory and factory.get_name()=='rtspclientsink':
            internal=element.get_by_name('rtspbin')
            if internal is None:raise RuntimeError('Missing rtspclientsink internal bin')
            previous=internal.get_property('async-handling')
            # The internal bin has locked_state=True; the parent cannot complete
            # its pending PLAYING transition. Let this bin complete it itself.
            internal.set_property('async-handling',True)
            report('rtsp_startup_fix',previous=previous,current=internal.get_property('async-handling'))
            fixed+=1
    if fixed!=1:raise RuntimeError('Expected exactly one RTSP publisher')
    stopping=[False]
    def stop(signum,frame):stopping[0]=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    bus=pipeline.get_bus();code=0
    try:
        if pipeline.set_state(Gst.State.PLAYING)==Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Pipeline refused PLAYING')
        mask=(Gst.MessageType.ERROR|Gst.MessageType.WARNING|Gst.MessageType.EOS|
              Gst.MessageType.PROGRESS|Gst.MessageType.STATE_CHANGED)
        while not stopping[0]:
            message=bus.timed_pop_filtered(100_000_000,mask)
            if message is None:continue
            source=message.src.get_name()
            if message.type==Gst.MessageType.ERROR:
                error,detail=message.parse_error();report('error',source=source,error=str(error),detail=detail);code=1;break
            if message.type==Gst.MessageType.WARNING:
                error,detail=message.parse_warning();report('warning',source=source,error=str(error),detail=detail)
            elif message.type==Gst.MessageType.EOS:report('eos');break
            elif message.type==Gst.MessageType.PROGRESS:
                kind,stage,text=message.parse_progress();report('progress',stage=stage,message=text)
            elif message.type==Gst.MessageType.STATE_CHANGED and (source=='rtspbin' or message.src==pipeline):
                old,new,pending=message.parse_state_changed()
                report('state',source=source,old=old.value_nick,new=new.value_nick,pending=pending.value_nick)
    finally:
        report('stopping');pipeline.set_state(Gst.State.NULL)
    return code


if __name__=='__main__':sys.exit(main(sys.argv[1:]))
