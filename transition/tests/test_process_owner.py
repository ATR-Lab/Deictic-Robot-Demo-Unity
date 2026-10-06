"""No ROS/robot required: shell-owned children receive real termination signals."""
from pathlib import Path
import shlex
import subprocess
import time
import pytest


@pytest.mark.parametrize('cleanup_delay', [0, 9])
def test_owned_cleanup_delivers_interrupt_and_keeps_unrelated_process(tmp_path, cleanup_delay):
    marker = tmp_path/'interrupt.txt'
    ready = tmp_path/'ready.txt'
    child = tmp_path/'child.py'
    child.write_text('import signal,time,pathlib,sys\n'
        'def stop(*args):\n'
        f' time.sleep({cleanup_delay})\n'
        ' pathlib.Path(sys.argv[1]).write_text("graceful")\n'
        ' raise SystemExit(0)\n'
        'signal.signal(signal.SIGINT,stop)\n'
        'pathlib.Path(sys.argv[2]).write_text("ready")\n'
        'while True: time.sleep(.1)\n')
    helper = Path(__file__).resolve().parents[1]/'scripts/process_owner.sh'
    script = ('set -euo pipefail\nsource '+shlex.quote(str(helper))+'\n'
              'start_owned_with_grace 30 /usr/bin/python3 '+ ' '.join(shlex.quote(str(p)) for p in (child,marker,ready))+'\n'
              'for i in {1..50}; do [[ -f '+shlex.quote(str(ready))+' ]] && break; sleep .1; done\n'
              'cleanup_owned\n')
    unrelated = subprocess.Popen(['/usr/bin/python3','-c','import time;time.sleep(30)'])
    try:
        started = time.monotonic()
        subprocess.run(['bash','-c',script],check=True,timeout=15)
        assert time.monotonic()-started < max(5,cleanup_delay+3)
        assert marker.read_text() == 'graceful'
        assert unrelated.poll() is None
    finally:
        unrelated.terminate(); unrelated.wait(timeout=3)
