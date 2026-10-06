#!/usr/bin/env bash
# Build only the official message/service definitions. Never runs vendor demos.
set -eo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
revision=5f9182e1a1b704f65e0065c83a3b6be2d066f3b3
vendor="$root/.codex/ros2/vendor/booster_robotics_sdk_ros2-20260924"
runtime="$root/transition/.runtime/vendor-ros2"
unset PYTHONHOME PYTHONPATH
source /opt/ros/jazzy/setup.bash
set -u
export PATH=/usr/bin:/bin:$PATH
if [[ ! -d "$vendor/.git" ]]; then
    git clone https://github.com/BoosterRobotics/booster_robotics_sdk_ros2.git "$vendor"
    git -C "$vendor" checkout --detach "$revision"
fi
[[ $(git -C "$vendor" rev-parse HEAD) == "$revision" ]] || { echo 'Unexpected vendor revision; use a separate checked-out dependency.' >&2; exit 1; }
/usr/bin/python3 - "$vendor" "$runtime" <<'PY'
import hashlib,json,subprocess,sys
from pathlib import Path
vendor,runtime=map(Path,sys.argv[1:]);runtime.mkdir(parents=True,exist_ok=True)
name='booster_ros2_interface/msg/Subtitle.msg'
changed=set(subprocess.check_output(['git','-C',str(vendor),'diff','--name-only','HEAD']).decode().splitlines())
untracked=subprocess.check_output(['git','-C',str(vendor),'ls-files','--others','--exclude-standard']).decode().splitlines()
if changed-{name} or untracked:
    raise SystemExit('Vendor checkout contains unrelated local changes; refusing to build')
original=subprocess.check_output(['git','-C',str(vendor),'show','HEAD:'+name])
digest=hashlib.sha256(original).hexdigest()
if digest!='9effe6bf56a52c41282cae091bc7ef3ce39bb0f56ec45873166acf99e9eeb0ae':
    raise SystemExit('Unexpected Subtitle.msg; refusing an unreviewed patch')
# Upstream used C++ semicolons/comments in this one .msg definition. Keep every
# field/type/order intact and translate only its syntax to valid ROS IDL input.
patched='\n'.join(line.split('//',1)[0].strip().removesuffix(';').rstrip()
                  for line in original.decode().splitlines())+'\n'
path=vendor/name
if path.read_bytes() not in (original,patched.encode()):
    raise SystemExit('Subtitle.msg contains unrelated local edits')
path.write_text(patched)
records={str(p.relative_to(vendor)):hashlib.sha256(p.read_bytes()).hexdigest()
         for p in sorted((vendor/'booster_ros2_interface').rglob('*')) if p.is_file()}
(runtime/'source-manifest.json').write_text(json.dumps({'revision':subprocess.check_output(['git','-C',str(vendor),'rev-parse','HEAD']).decode().strip(),
    'local_patch':{'file':name,'before_sha256':digest,'after_sha256':hashlib.sha256(patched.encode()).hexdigest(),
                   'reason':'ROS msg fields cannot contain C++ semicolons or // comments'},'files':records},indent=2))
PY
cd "$root"
colcon --log-base "$runtime/log" build --base-paths "$vendor/booster_ros2_interface" \
  --build-base "$runtime/build" --install-base "$runtime/install" --packages-select booster_interface \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
echo "Source $runtime/install/setup.bash after ROS Jazzy when using the observer. No vendor executable was run."
