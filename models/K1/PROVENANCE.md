# Booster K1 vendor assets

Source: <https://github.com/BoosterRobotics/booster_assets/tree/3c2dfa99e09beddf092e0d6521dbbcec7e7903ed/robots/K1>

Revision: `3c2dfa99e09beddf092e0d6521dbbcec7e7903ed`, retrieved 2026-09-16.
The unmodified `K1_22dof.urdf` and its 24 referenced STL files are included under
the upstream BSD 3-Clause license in `LICENSE`. `sim/fetch_k1_assets.py` restores
these exact files. The locomotion variant fixes the arms and is unsuitable here.

K1 has four joints per arm, no wrist articulation and no gripper. Deictic reaching
uses right-arm position IK; bimanual teleoperation uses position-prioritized pose
IK for both arms. The virtual tool tips are 0.10 m along terminal local Y: negative
Y on `right_elbow_yaw_link`, positive Y on `left_elbow_yaw_link`. These are demo
tool definitions, not vendor end-effector calibrations.

The simulated right-wrist camera mount is an adaptation, not a physical camera
specified by Booster's URDF. The separate head-camera pair uses the URDF's
`head_booster_stereo_rgb_link` origin with a provisional 64 mm virtual baseline;
its eye offsets and intrinsics are not physical K1 calibration. See the
[simulation camera description](../../sim/README.md#robot-head-stereo-display).

`base_link` is an alias for `trunk`. The simulator derives a temporary URDF with
both four-joint arms movable and all other revolute joints fixed. It never edits
the vendor URDF.
