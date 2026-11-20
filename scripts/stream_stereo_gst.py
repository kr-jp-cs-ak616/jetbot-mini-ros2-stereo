#!/usr/bin/env python3
"""JetPack host capture; sends JPEG + shared-clock stamps to local ROS bridge."""
import argparse
from pathlib import Path
import signal
import socket
import sys
import threading
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'/'jetbot_stereo'))
from jetbot_stereo.protocol import pack_frame
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst


def connect_receiver(port,timeout=30):
    deadline=time.monotonic()+timeout
    while True:
        try:
            return socket.create_connection(('127.0.0.1',port),timeout=1)
        except OSError:
            if time.monotonic()>=deadline:
                raise RuntimeError('ROS camera receiver did not become ready on port {}'.format(port))
            time.sleep(.5)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    Gst.init(None)
    connection=connect_receiver(args.port)
    connection.settimeout(3)
    branches=[]
    for sensor in (0,1):
        branches.append(
            'nvarguscamerasrc sensor-id={} ! '
            'video/x-raw(memory:NVMM),width=640,height=480,framerate=15/1,format=NV12 ! '
            'nvvidconv ! video/x-raw,format=I420 ! jpegenc ! '
            'appsink name=sink{} emit-signals=true drop=true max-buffers=1 sync=false'.format(sensor,sensor))
    pipeline=Gst.parse_launch(' '.join(branches))
    lock=threading.Lock()
    stopped=threading.Event()
    counts=[0,0]
    errors=[]
    for sensor in (0,1):
        def sample(sink,sensor=sensor):
            value=sink.emit('pull-sample')
            if value is None:
                return Gst.FlowReturn.EOS
            buf=value.get_buffer()
            running=value.get_segment().to_running_time(Gst.Format.TIME,buf.pts)
            if running==Gst.CLOCK_TIME_NONE:
                return Gst.FlowReturn.OK
            clock=pipeline.get_clock()
            capture_time=pipeline.get_base_time()+running
            # Ubuntu 18.04's Python 3.6 has no time.time_ns().
            stamp=int(time.time()*1000000000)-(clock.get_time()-capture_time)
            jpeg=buf.extract_dup(0,buf.get_size())
            try:
                with lock:
                    connection.sendall(pack_frame(sensor,stamp,counts[sensor],jpeg))
                counts[sensor]+=1
            except (OSError,ValueError) as exc:
                errors.append(str(exc));stopped.set()
                return Gst.FlowReturn.ERROR
            return Gst.FlowReturn.OK
        pipeline.get_by_name('sink{}'.format(sensor)).connect('new-sample',sample)
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:stopped.set())
    try:
        if pipeline.set_state(Gst.State.PLAYING)==Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Cannot start CSI pipeline')
        bus=pipeline.get_bus()
        while not stopped.is_set():
            message=bus.timed_pop_filtered(Gst.SECOND,Gst.MessageType.ERROR|Gst.MessageType.EOS)
            if message is not None:
                if message.type==Gst.MessageType.ERROR:
                    error,debug=message.parse_error()
                    raise RuntimeError('{}: {}'.format(error,debug))
                break
        if errors:
            raise RuntimeError(errors[-1])
    finally:
        pipeline.set_state(Gst.State.NULL)
        connection.close()
        print('Sent left/right frames:',counts,flush=True)


if __name__=='__main__':
    main()
