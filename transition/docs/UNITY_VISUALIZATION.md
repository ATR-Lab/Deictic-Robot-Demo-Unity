# Reference Isaac task visualization

The transition worker publishes a receive-only visualization stream when started
with `--ros`. The shell launcher enables it by default and selects ROS domain174
with UDPv4 transport and default discovery. This separates the reference task from the existing
domain42 teleoperation demonstration and from the physical robot's vendor DDS
network. All workstation visualization processes must use the same domain.

The reference robot retains the supplied task's fixed trunk, left arm, head and
legs. Only four right-arm joints articulate. Head stereo images come from the
same simulated K1 and scene; the neck does not follow the headset in this task
mode. These virtual cameras are not a calibration claim for physical cameras.

| Topic | Message | Meaning |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Four measured right-arm positions; finite-difference velocities after the first advancing sample |
| `/transition/simulation/telemetry` | `std_msgs/String` JSON | Boot/sample/source timestamps, native and derived velocity, fixed-model provenance |
| `/k1/head_camera/left/image_raw` | `sensor_msgs/Image` | Left simulated eye, 320×240 RGB |
| `/k1/head_camera/right/image_raw` | `sensor_msgs/Image` | Right simulated eye, synchronized renderer reference |
| `/k1/head_camera/{left,right}/camera_info` | `sensor_msgs/CameraInfo` | Matching virtual calibration and stamp |

Joint names are `aaright_shoulder_pitch_joint`, `right_shoulder_roll_joint`,
`right_elbow_pitch_joint`, and `right_elbow_yaw_joint`. No left/head/leg readings
are manufactured to fill a 22-joint array. The Unity model may render those fixed
parts from the model, while displaying the measurement scope explicitly.

There are no ROS command subscriptions, trajectory actions or motion services.
The task runtime's loopback HTTP connection remains the only worker command
path. Do not start the old IK controller/trajectory relay alongside this mode.

From the complete monorepo on MLWorkstation:

```sh
ROS_DOMAIN_ID=174 bash transition/scripts/run_isaac_worker.sh
```

The launcher mounts the complete checkout so shared `sim/head_stereo.py` and
`camera_sync.py` can be reused. It creates a unique `.runtime/isaac-run-*` output
directory for each launch; preserve its journal. An explicit `--output` overrides
that choice. Direct worker invocation defaults to no ROS; `--no-ros` also disables
it through the launcher.

The existing workstation camera relay converts demanded synchronized eye pairs
to the Unity SBS JPEG topic. In a second terminal:

```sh
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=174
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
/usr/bin/python3 ros2/scripts/camera_view_relay.py
```

The eye render products remain disabled until a subscriber requests their data.
The relay likewise acquires sources only when Unity requests the composed stream.
Publication caps are not guaranteed delivered frame rates. Metadata distinguishes
simulation source time from post-step host receipt/ROS timestamp. Eye stamps use
the continuous spectator camera's matching cached renderer reference; they are
not renewed at later JPEG publication.

Start only the receive-only ROS-TCP endpoint for this visualization mode. The
ordinary deictic bridge launcher also enables command producers and is unsuitable
here. Operator controls use the separate [HTTP API](UNITY_OPERATOR_API.md).
