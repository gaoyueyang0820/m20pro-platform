#!/usr/bin/env python3
"""Deterministic locked-bin state regression using the robot's installed GST.

Synthetic appsrc/fakesink only: no camera, RTSP connection or robot command.
Requires PyGObject/GStreamer, already installed on the investigated AOS.
"""
import json
import time


def reproduce(fixed):
    import gi
    gi.require_version('Gst','1.0')
    from gi.repository import Gst
    Gst.init(None)
    pipeline=Gst.Pipeline.new('outer');inner=Gst.Bin.new('locked')
    inner.set_property('async-handling',fixed);inner.set_locked_state(True)
    src=Gst.ElementFactory.make('appsrc','input');src.set_property('format',Gst.Format.TIME)
    sink=Gst.ElementFactory.make('fakesink','output');sink.set_property('sync',False);sink.set_property('async',True)
    inner.add(src);inner.add(sink);src.link(sink);pipeline.add(inner)
    try:
        pipeline.set_state(Gst.State.PAUSED);inner.set_state(Gst.State.PAUSED)
        pipeline.set_state(Gst.State.PLAYING);inner.set_state(Gst.State.PLAYING)
        # Complete preroll after PLAYING was requested, reproducing the race.
        buf=Gst.Buffer.new_allocate(None,16,None);buf.pts=0;buf.duration=Gst.SECOND
        src.emit('push-buffer',buf);time.sleep(.4)
        return dict(fixed=fixed,outer=pipeline.get_state(0)[1].value_nick,inner=inner.get_state(0)[1].value_nick)
    finally:
        inner.set_state(Gst.State.NULL);pipeline.set_state(Gst.State.NULL)


if __name__=='__main__':
    before,after=reproduce(False),reproduce(True)
    print(json.dumps(dict(before=before,after=after)))
    assert before['outer']=='playing' and before['inner']=='paused'
    assert after['outer']=='playing' and after['inner']=='playing'
