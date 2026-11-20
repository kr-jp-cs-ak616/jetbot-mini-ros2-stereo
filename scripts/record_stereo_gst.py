#!/usr/bin/env python3
"""JetPack host: record two CSI streams in one shared-clock GStreamer pipeline."""
import csv
import json
from pathlib import Path
import sys
import time

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


def main():
    folder=Path(sys.argv[1])
    seconds=int(sys.argv[2])
    folder.mkdir(parents=True,exist_ok=True)
    Gst.init(None)
    branches=[]
    for sensor in (0,1):
        branches.append(
            'nvarguscamerasrc sensor-id={} num-buffers={} ! '
            'video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! '
            'nvvidconv ! video/x-raw,format=I420 ! jpegenc name=encoder{} ! '
            'avimux ! filesink location="{}"'.format(
                sensor,seconds*15,sensor,folder/('camera-{}.avi'.format(sensor))))
    pipeline=Gst.parse_launch(' '.join(branches))
    files=[]
    counts=[0,0]
    start=time.time()
    try:
        for sensor in (0,1):
            stream=(folder/('camera-{}-timestamps.csv'.format(sensor))).open('w',newline='')
            files.append(stream)
            writer=csv.writer(stream)
            writer.writerow(['frame_index','pts_ns','running_time_ns','clock_time_ns'])
            def probe(pad,info,sensor=sensor,writer=writer):
                buf=info.get_buffer()
                segment_event=pad.get_sticky_event(Gst.EventType.SEGMENT,0)
                running=Gst.CLOCK_TIME_NONE
                if segment_event is not None and buf.pts!=Gst.CLOCK_TIME_NONE:
                    running=segment_event.parse_segment().to_running_time(Gst.Format.TIME,buf.pts)
                clock_time=(pipeline.get_base_time()+running) if running!=Gst.CLOCK_TIME_NONE else -1
                writer.writerow([counts[sensor],int(buf.pts),int(running),int(clock_time)])
                counts[sensor]+=1
                return Gst.PadProbeReturn.OK
            pipeline.get_by_name('encoder{}'.format(sensor)).get_static_pad('src').add_probe(
                Gst.PadProbeType.BUFFER,probe)
        if pipeline.set_state(Gst.State.PLAYING)==Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Cannot start CSI recording')
        message=pipeline.get_bus().timed_pop_filtered(
            (seconds+25)*Gst.SECOND,Gst.MessageType.ERROR|Gst.MessageType.EOS)
        if message is None:
            raise RuntimeError('Recording timeout')
        if message.type==Gst.MessageType.ERROR:
            error,debug=message.parse_error()
            raise RuntimeError('{}: {}'.format(error,debug))
        if min(counts)==0:
            raise RuntimeError('One camera produced no frames')
        manifest=dict(requested_seconds=seconds,fps=15,width=640,height=480,
                      frames=counts,started_unix_s=start,finished_unix_s=time.time(),
                      timing='Shared Gst pipeline clock; buffer presentation timestamps, not proven exposure synchronization',
                      hardware_synchronized=False)
        (folder/'recording.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print('Recorded frames:',counts)
    finally:
        pipeline.set_state(Gst.State.NULL)
        for stream in files:
            stream.close()


if __name__=='__main__':
    main()
