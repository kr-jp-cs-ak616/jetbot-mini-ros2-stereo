#!/usr/bin/env python3
"""JetBot host control for the camera bridge only; leaves motor driver alone."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
CONTAINER='jetbot-mini-ros2'
PIDFILE='/tmp/jetbot-stereo-publisher.pid'
PUBLOG='/tmp/jetbot-stereo-publisher.log'


def docker(args,detached=False):
    return subprocess.run(['docker','exec']+(['-d'] if detached else [])+[CONTAINER]+args,
                          check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                          universal_newlines=True)


def process_pid(path,needle):
    try:
        pid=int(Path(path).read_text().strip())
        if needle not in Path('/proc/{}/cmdline'.format(pid)).read_bytes().decode(errors='replace'):
            return None
        os.kill(pid,0)
        return pid
    except (OSError,ValueError):
        return None


def publisher_pid():
    code=('from pathlib import Path; import os; '
          'p=int(Path("'+PIDFILE+'").read_text()); '
          'assert "jetbot_stereo.socket_camera" in Path("/proc/%s/cmdline"%p).read_text(); '
          'os.kill(p,0); print(p)')
    try:
        return int(docker(['python3','-c',code]).stdout.strip())
    except (subprocess.CalledProcessError,ValueError):
        return None


def publisher_listening():
    # Capture holds the accepted connection for its entire lifetime. Connecting
    # here fills the unconsumed accept backlog on repeated start commands.
    try:
        rows=Path('/proc/net/tcp').read_text().splitlines()[1:]
        return any(len(fields)>3 and fields[1]=='0100007F:223D' and fields[3]=='0A'
                   for fields in (row.split() for row in rows))
    except OSError:
        return False


def wait_for_publisher(timeout=30):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if publisher_pid() is not None and publisher_listening():
            return
        time.sleep(.25)
    raise RuntimeError('ROS camera receiver did not open port 8765; check inside '
                       +CONTAINER+': '+PUBLOG)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start','stop','status'])
    args=parser.parse_args()
    capture_file=ROOT/'capture.pid'
    capture_needle=str(ROOT/'scripts/stream_stereo_gst.py')
    if args.action=='start':
        running=subprocess.check_output(['docker','inspect','--format','{{.State.Running}}',CONTAINER],universal_newlines=True).strip()
        if running!='true':
            subprocess.run(['docker','start',CONTAINER],check=True)
        try:
            docker(['test','-f','/opt/jetbot-stereo/src/jetbot_stereo/jetbot_stereo/socket_camera.py'])
        except subprocess.CalledProcessError:
            subprocess.run(['docker','exec','-u','0',CONTAINER,'mkdir','-p','/opt/jetbot-stereo'],check=True)
            subprocess.run(['docker','cp',str(ROOT)+'/.',CONTAINER+':/opt/jetbot-stereo'],check=True)
        if publisher_pid() is None:
            script=('source /opt/ros/humble/setup.bash; '
                    'export PYTHONPATH="/opt/jetbot-stereo/src/jetbot_stereo:${PYTHONPATH:-}"; '
                    'echo $$ > '+PIDFILE+'; '
                    'exec python3 -u -m jetbot_stereo.socket_camera --ros-args '
                    '-p calibration_dir:=/opt/jetbot-stereo/calibration > '+PUBLOG+' 2>&1')
            docker(['bash','-lc',script],detached=True)
        wait_for_publisher()
        if process_pid(capture_file,capture_needle) is None:
            with (ROOT/'capture.log').open('w') as log:
                process=subprocess.Popen([sys.executable,'-u',capture_needle],
                                         stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                                         start_new_session=True)
            capture_file.write_text(str(process.pid)+'\n')
            time.sleep(1)
            if process.poll() is not None:
                raise RuntimeError('CSI capture exited; check '+str(ROOT/'capture.log'))
        print('Camera capture and ROS publisher started. ROS_DOMAIN_ID=16')
    elif args.action=='stop':
        pid=process_pid(capture_file,capture_needle)
        if pid is not None:
            os.kill(pid,signal.SIGTERM)
            time.sleep(2)
        pid=publisher_pid()
        if pid is not None:
            docker(['python3','-c','import os,signal; os.kill({},signal.SIGTERM)'.format(pid)])
            deadline=time.monotonic()+10
            while publisher_pid() is not None:
                if time.monotonic()>=deadline:
                    raise RuntimeError('Camera publisher did not stop within 10 seconds')
                time.sleep(.25)
        print('Camera processes stopped. Motor driver was not changed.')
    else:
        print('capture_pid:',process_pid(capture_file,capture_needle))
        print('publisher_pid:',publisher_pid())
        try:
            print(docker(['tail','-n','4',PUBLOG]).stdout)
        except subprocess.CalledProcessError:
            pass


if __name__=='__main__':
    main()
