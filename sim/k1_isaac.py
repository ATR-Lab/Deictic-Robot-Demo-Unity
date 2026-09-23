#!/usr/bin/env python3
"""Fixed-base K1 reaching scene for Isaac Sim 5.0; never connects to hardware."""
import argparse
import json
from pathlib import Path
import tempfile
import time

from k1_model import BOTH_ARM_JOINTS, HEAD_JOINTS, UPPER_BODY_JOINTS, TOOL_OFFSET, URDF, fixed_arm_urdf, joint_limits
from command_control import ArmSetpoints
from head_control import HeadSetpoints, MAX_COMMAND_BYTES
from loop_timing import RealtimeSchedule, drain_callbacks

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--no-ros", action="store_true", help="Import/physics/render smoke test without ROS")
parser.add_argument("--smoke", action="store_true", help="Move both simulated arms and verify measured response")
parser.add_argument("--synthetic-headset", action="store_true", help="Publish a second rendered RGBD camera and known headset-world pose")
head_stereo_options = parser.add_mutually_exclusive_group()
head_stereo_options.add_argument("--head-stereo", dest="head_stereo", action="store_true",
                                help="Enable demand-rendered robot-forward stereo display (default with ROS)")
head_stereo_options.add_argument("--no-head-stereo", dest="head_stereo", action="store_false",
                                help="Disable the optional head stereo display cameras")
parser.set_defaults(head_stereo=None)
parser.add_argument("--head-stereo-baseline", type=float, default=.064,
                    help="Assumed simulation-only stereo baseline in metres; not physical calibration")
parser.add_argument("--head-stereo-width", type=int, choices=(320,640), default=320,
                    help="Per-eye display width, 4:3 aspect; 320 reduces stereo RTX cost")
parser.add_argument("--head-stereo-rate", type=float, default=15.,
                    help="Newest coherent stereo publication cap, 1..30 Hz; render products stay warm on demand")
parser.add_argument("--headset-resolution-scale", type=int, choices=(1,2), default=1,
                    help="Native headset RGBD render scale: 1=640x480, 2=1280x960; field of view is unchanged")
parser.add_argument("--webrtc", action="store_true", help="Enable Isaac Sim 5.0 WebRTC streaming on TCP49100/UDP47998")
parser.add_argument("--public-ip", default="", help="Server IP reachable by the WebRTC client")
parser.add_argument("--steps", type=int, default=0, help="Stop after N physics steps; zero runs continuously")
parser.add_argument("--legacy-loop-timing", action="store_true",
                    help="Diagnostic comparison: historical render-every-four-step timing without wall-time catch-up")
parser.add_argument("--output", type=Path, default=Path(__file__).parent / "artifacts")
parser.add_argument("--tool-offset", nargs=3, type=float, default=TOOL_OFFSET)
parser.add_argument("--camera-offset", nargs=3, type=float, default=(0.0, -0.10, 0.08))
parser.add_argument("--camera-pitch", type=float, default=1.0, help="Virtual wrist mount downward tilt in radians")
parser.add_argument("--camera-roll", type=float, default=-0.804, help="Fixed optical-axis mount roll; levels horizon at the rest pose")
parser.add_argument("--camera-mount-profile", choices=("rest", "reach-balanced", "reach-balanced-10cm", "reach-projection-balanced"), default="rest",
                    help="Select a fixed virtual camera mount; rest uses offset/pitch/roll; other profiles preserve documented geometric experiments")
parser.add_argument("--textured-table", action="store_true", help="Add nonrepeating procedural print for visual-registration experiments")
parser.add_argument("--floor-profile", choices=("grid", "matte"), default="grid",
                    help="grid preserves the Isaac default; matte replaces only its visual material with uniform diffuse gray")
args = parser.parse_args()
if not 1. <= args.head_stereo_rate <= 30.:
    parser.error("--head-stereo-rate must be in [1, 30] Hz")

from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless, "hide_ui": not args.webrtc,
                     "renderer": "RaytracedLighting", "width": 1280, "height": 720})

import numpy as np
import omni.kit.commands
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, VisualSphere
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.rotations import quat_to_rot_matrix, rot_matrix_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from camera_sync import snapshot, synchronized


def configure_camera(camera):
    # Configure after initialize, as required by Isaac 5.0's calibrated-camera
    # example. Explicit OpenCV intrinsics prevent the renderer using its default
    # lens while USD getters report separately authored physical lens values.
    camera.set_focal_length(.012)
    camera.set_horizontal_aperture(.024)
    camera.set_vertical_aperture(.018)
    camera.set_focus_distance(.5)
    camera.set_lens_aperture(0.)
    camera.set_clipping_range(.015,20.)
    width,height = camera.get_resolution()
    scale = width/640.
    camera.set_opencv_pinhole_properties(cx=320.*scale,cy=240.*scale,
                                         fx=320.*scale,fy=320.*scale,pinhole=[0.]*8)
    # The schema defaults to calibration imageSize=(2048,1024). Isaac 5.0's
    # helper does not author it, so RTX would scale the pixel intrinsics again.
    image_size = camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize")
    if not image_size:
        raise RuntimeError("Renderer lacks the calibrated OpenCV imageSize attribute")
    image_size.Set(Gf.Vec2i(width,height))


def intrinsics(camera):
    calibration_size = tuple(camera.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get())
    if calibration_size != tuple(camera.get_resolution()):
        raise RuntimeError(f"Camera calibration size {calibration_size} differs from render size {camera.get_resolution()}")
    cx,cy,fx,fy,_ = camera.get_opencv_pinhole_properties()
    return np.array([[fx,0.,cx],[0.,fy,cy],[0.,0.,1.]],dtype=float)


def capture_pair(output, wrist, headset, frames=None):
    """Export calibrated synchronized renderer data for independent visual tests."""
    from PIL import Image as PILImage
    wrist_frame,head_frame = frames or (snapshot(wrist),snapshot(headset,require_depth=True))
    if not synchronized(wrist_frame,head_frame):
        return False
    PILImage.fromarray(head_frame.rgb).save(output/"headset_camera.png")
    PILImage.fromarray(wrist_frame.rgb).save(output/"wrist_camera.png")
    np.save(output/"headset_depth_z.npy",head_frame.depth)
    def matrix(frame,offset):
        value = frame.base_from_optical.copy()
        value[:3,3] += offset
        return value.tolist()
    metadata = {"headset_K":intrinsics(headset).tolist(),
                "wrist_K":intrinsics(wrist).tolist(),
                "headset_calibration_image_size":list(headset.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()),
                "wrist_calibration_image_size":list(wrist.prim.GetAttribute("omni:lensdistortion:opencvPinhole:imageSize").Get()),
                "T_world_headOptical":matrix(head_frame,np.array([1.2,0.,.9])),
                "T_base_wristOptical":matrix(wrist_frame,np.zeros(3)),
                "T_base_headsetWorld":[[1,0,0,-1.2],[0,1,0,0],[0,0,1,-.9],[0,0,0,1]],
                "depth_encoding":"32FC1 optical Z meters", "synthetic":True,
                "rendering_time_seconds":head_frame.rendering_time,
                "render_reference":list(head_frame.reference),
                "pose_source":"CameraParams in same cached renderer frame as RGB/depth",
                "fixture":"coarse_nonperiodic_head_clear_leveled_wrist" if args.textured_table else "geometry_only",
                "camera_mount_profile":args.camera_mount_profile,
                "floor_profile":args.floor_profile,
                "headset_resolution_scale":args.headset_resolution_scale,
                "camera_acquisition":"every actual render; coherent new pairs capped at 15 Hz wall time",
                "capture_wall_time_unix":time.time()}
    (output/"camera_pair.json").write_text(json.dumps(metadata,indent=2))
    return True


def publish_pose(publisher, position, quaternion, stamp, frame="base_link"):
    message = PoseStamped()
    message.header.frame_id = frame
    message.header.stamp = stamp
    message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, position)
    message.pose.orientation.w, message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z = map(float, quaternion)
    publisher.publish(message)


def main():
    args.output.mkdir(parents=True, exist_ok=True)
    if args.webrtc:
        app.set_setting("/app/window/drawMouse", True)
        app.set_setting("/app/livestream/allowDynamicResize", True)
        if args.public_ip:
            app.set_setting("/app/livestream/publicEndpointAddress", args.public_ip)
        app.set_setting("/app/livestream/port", 49100)
        enable_extension("omni.services.livestream.nvcf")
    enable_extension("isaacsim.asset.importer.urdf")
    from isaacsim.asset.importer.urdf import _urdf
    if not args.no_ros:
        enable_extension("isaacsim.ros2.bridge")
    app.update()
    world = World(stage_units_in_meters=1.0, physics_dt=1/120, rendering_dt=1/30)
    ground = world.scene.add_default_ground_plane(z_position=-0.70)
    if args.floor_profile == "matte":
        from isaacsim.core.api.materials import PreviewSurface
        # Keep the existing collision geometry and physics material; override only
        # the default grid asset's visual material, including child bindings.
        floor_material = PreviewSurface("/World/Looks/MatteFloor", color=np.array([.35,.35,.35]),
                                        roughness=1.0, metallic=0.0)
        ground.xform_prim.apply_visual_material(floor_material, weaker_than_descendants=False)
    # Pelvis/trunk is the world origin, so ROS base_link and Isaac world coincide.
    with tempfile.TemporaryDirectory(prefix="deictic-k1-") as temporary:
        derived = Path(temporary) / "K1_fixed_reaching.urdf"
        fixed_arm_urdf(derived)
        _, config = omni.kit.commands.execute("URDFCreateImportConfig")
        config.merge_fixed_joints = False
        config.fix_base = True
        config.import_inertia_tensor = True
        config.convex_decomp = False
        config.self_collision = False
        config.default_drive_strength = 80.0
        config.default_position_drive_damping = 4.0
        config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        success, root_path = omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=str(derived), import_config=config,
            get_articulation_root=True,
        )
    if not success:
        raise RuntimeError("K1 URDF import failed")
    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1500)
    world.scene.add(FixedCuboid("/World/Table", name="table", position=np.array([.28, -.30, -.17]),
                                scale=np.array([.55, .50, .04]), color=np.array([.55,.44,.30])))
    if args.textured_table:
        from table_texture import add_textured_surface
        add_textured_surface(stage,args.output/"table_texture.png")
    for i in range(5):
        world.scene.add(VisualSphere(f"/World/Target{i}", name=f"target{i}",
                                    position=np.array([.08+.02*i,-.25,.02]), radius=.0125,
                                    color=np.array([.15,.7,1.])))
    # Reproducible colored geometry gives rendered cameras nonuniform features.
    rng = np.random.default_rng(7)
    for i in range(35):
        position = np.array([.10+(i%7)*.06, -.44+(i//7)*.11, -.139])
        yaw = rng.uniform(-np.pi,np.pi)
        world.scene.add(FixedCuboid(f"/World/Feature{i}", name=f"feature{i}", position=position,
                                    scale=np.array([rng.uniform(.015,.035),rng.uniform(.02,.065),rng.uniform(.008,.028)]),
                                    orientation=np.array([np.cos(yaw/2),0.,0.,np.sin(yaw/2)]),
                                    color=np.repeat(rng.uniform(.03,.95),3)))
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.RevoluteJoint):
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(80.0)
            drive.CreateDampingAttr(4.0)
            drive.CreateMaxForceAttr(6.0 if prim.GetName() in HEAD_JOINTS else 14.0)
    matches = [prim for prim in stage.Traverse() if prim.GetName() == "right_elbow_yaw_link"
               and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one terminal link; found {len(matches)}")
    terminal_path = str(matches[0].GetPath())
    terminal = SingleXFormPrim(terminal_path, name="terminal")
    # Keep every actual render: Isaac 5.0's Camera callback treats negative
    # frequency as unthrottled acquisition. Publication has its own wall-time
    # cap below, so slow renders are not decimated again by simulation time.
    camera = Camera(terminal_path+"/WristCamera", name="wrist_camera", frequency=-1, resolution=(640,480))
    # Quaternion is wxyz; ROS optical +Z looks along terminal +X tilted downward.
    c,s = np.cos(args.camera_pitch/2),np.sin(args.camera_pitch/2)
    mount = quat_to_rot_matrix(np.array([.5*(c-s),-.5*(c+s),.5*(c+s),-.5*(c-s)]))
    cr,sr = np.cos(args.camera_roll),np.sin(args.camera_roll)
    mount = mount @ np.array([[cr,-sr,0],[sr,cr,0],[0,0,1]])
    camera_offset = np.array(args.camera_offset)
    if args.camera_mount_profile in ("reach-balanced", "reach-balanced-10cm"):
        # A fixed external side bracket, not an adaptive look-at camera. The
        # terminal mesh spans X +/-0.03 m; X=0.06 keeps the steep optical ray
        # outside the hand. Clocking preserves the original image-right direction.
        # The 10 cm profile maximizes the minimum shared, mesh-unoccluded table
        # area across rest and five sampled reach paths. Existing profiles remain
        # reproducible; neither the camera pose nor intrinsics adapt during motion.
        bracket_x = .10 if args.camera_mount_profile == "reach-balanced-10cm" else .06
        camera_offset = np.array([bracket_x,-.10,.08])
        mount = quat_to_rot_matrix(np.array([.221391832444005367,.8673395077793156,
                                             -.4396883649539208,.0733619553864093]))
    elif args.camera_mount_profile == "reach-projection-balanced":
        # Selected by fixed mesh visibility and weakest-axis projected feature
        # support over 107 poses, before any images from this profile were scored.
        # Aim was computed once at 75% of the nominal middle-target trajectory.
        camera_offset = np.array([.14,-.10,.12])
        mount = quat_to_rot_matrix(np.array([.4199440579524442,.7831930522152495,
                                             -.45477144444863005,-.058639274409426203]))
    camera.set_local_pose(translation=camera_offset,
                          orientation=rot_matrix_to_quat(mount), camera_axes="ros")
    headset = None
    if args.synthetic_headset:
        eye, target = np.array([.05,-.65,.45]), np.array([.28,-.30,-.15])
        forward = (target-eye)/np.linalg.norm(target-eye)
        right = np.cross(forward, np.array([0.,0.,1.])); right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        headset = Camera("/World/SyntheticHeadsetCamera", name="synthetic_headset", frequency=-1,
                         resolution=(640*args.headset_resolution_scale,480*args.headset_resolution_scale))
        headset.set_world_pose(eye, rot_matrix_to_quat(np.column_stack((right,down,forward))), camera_axes="ros")
    robot = world.scene.add(SingleArticulation(root_path, name="k1"))
    set_camera_view(eye=np.array([1.7,-1.7,1.0]), target=np.array([0,0,-.05]))
    world.reset()
    camera.initialize()
    configure_camera(camera)
    camera.attach_annotator("CameraParams")
    if headset:
        headset.initialize()
        configure_camera(headset)
        headset.attach_annotator("CameraParams")
        headset.add_distance_to_image_plane_to_frame()
    names = robot.dof_names
    if set(names) != set(UPPER_BODY_JOINTS):
        raise RuntimeError(f"Fixed-base upper-body importer produced unexpected DOFs: {names}")
    indices = [names.index(name) for name in BOTH_ARM_JOINTS]
    head_indices = [names.index(name) for name in HEAD_JOINTS]
    upper_body_indices = indices + head_indices
    # Preserve the old fixture: the formerly locked left arm starts at zero;
    # the right arm keeps its validated rest pose. Neck starts forward; legs stay fixed.
    initial = np.array([0., 0., 0., 0., 0., 0.5, 0., 0.])
    robot.set_joint_positions(initial, joint_indices=indices)
    robot.set_joint_velocities(np.zeros(8), joint_indices=indices)
    robot.set_joint_positions(np.zeros(2), joint_indices=head_indices)
    robot.set_joint_velocities(np.zeros(2), joint_indices=head_indices)
    initial_state = robot.get_joint_positions()
    initial = np.array(initial_state[indices], dtype=float)
    desired = initial.copy()
    limits = joint_limits()
    setpoints = ArmSetpoints(initial.tolist(), limits)
    head_setpoints = HeadSetpoints(initial_state[head_indices].tolist(), limits)
    head_desired = np.asarray(head_setpoints.desired)
    node = None
    executor = None
    if not args.no_ros:
        global PoseStamped
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState, Image, CameraInfo
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("k1_fixed_base_isaac")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        state_pub = node.create_publisher(JointState, "/joint_states", 10)
        ee_pub = node.create_publisher(PoseStamped, "/k1/end_effector_pose", 10)
        camera_pose_pub = node.create_publisher(PoseStamped, "/k1/wrist_camera/pose", 10)
        image_pub = node.create_publisher(Image, "/k1/wrist_camera/image_raw", 2)
        info_pub = node.create_publisher(CameraInfo, "/k1/wrist_camera/camera_info", 2)
        head_status_pub = node.create_publisher(String, "/k1/head/status", 1)
        if headset:
            head_image_pub = node.create_publisher(Image, "/deictic/headset/image_raw", 2)
            head_depth_pub = node.create_publisher(Image, "/deictic/headset/depth", 2)
            head_info_pub = node.create_publisher(CameraInfo, "/deictic/headset/camera_info", 2)
            head_pose_pub = node.create_publisher(PoseStamped, "/deictic/headset/camera_pose", 10)

        def command(message):
            try:
                stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
                setpoints.command(message.name, message.position, stamp, time.monotonic(),
                                  node.get_clock().now().nanoseconds*1e-9)
            except (ValueError, TypeError) as error:
                node.get_logger().error(str(error))
        # Rendering may run slower than the relay: only the latest stamped
        # setpoint may cross the actuator boundary, never an old buffered queue.
        node.create_subscription(JointState, "/k1/sim/joint_commands", command, 1)

        def head_command(message):
            try:
                if len(message.data.encode("utf-8")) > MAX_COMMAND_BYTES:
                    raise ValueError("Head command exceeds bounded JSON payload size")
                head_setpoints.command(json.loads(message.data), time.monotonic(),
                                       node.get_clock().now().nanoseconds*1e-9)
            except (ValueError, TypeError, OverflowError, RecursionError) as error:
                node.get_logger().error(str(error))
        node.create_subscription(String, "/k1/head/command", head_command, 1)
    head_stereo = None
    enable_head_stereo = args.head_stereo if args.head_stereo is not None else not args.no_ros
    if enable_head_stereo:
        from head_stereo import HeadStereoDisplay
        head_stereo = HeadStereoDisplay(stage, args.output, node, configure_camera, intrinsics,
                                        args.head_stereo_baseline, args.head_stereo_width, args.head_stereo_rate)
    print("DEICTIC_K1_READY " + json.dumps({"joints": names, "base_frame": "base_link", "robot_root": root_path}), flush=True)
    first = None
    rendered = None
    completed = 0
    frame_count = 0
    pair_captured = False
    last_camera_reference = None
    next_camera_publication = 0.0
    next_head_status = 0.0
    timing = RealtimeSchedule(time.monotonic(), float(world.current_time))
    next_timing_diagnostic = 0.0
    # Constant-size aggregates expose CPU/GPU synchronization cost without
    # logging every physics iteration or retaining individual measurements.
    profile_sections = {}
    def profile_section(name, started):
        elapsed = time.monotonic()-started
        total, count, maximum = profile_sections.get(name, (0., 0, 0.))
        profile_sections[name] = (total+elapsed, count+1, max(maximum, elapsed))
    tool_offset = np.asarray(args.tool_offset)
    camera_start_deadline = time.monotonic()+30.
    try:
        while app.is_running() and (not args.steps or completed < args.steps):
            begin = time.monotonic()
            if frame_count == 0 and begin > camera_start_deadline:
                raise RuntimeError("No synchronized cached RGB/depth/CameraParams frames after 30 seconds")
            if executor:
                # Depth-one subscriptions plus a bounded drain give both head
                # and arm commands a turn before any post-render catch-up.
                section_started = time.monotonic()
                drain_callbacks(executor.spin_once)
                profile_section("ros_callbacks", section_started)
            if args.smoke or args.legacy_loop_timing:
                render_this_step = completed % 4 == 0
            else:
                physics_due, render_this_step, delay = timing.plan(time.monotonic(), float(world.current_time))
                if not physics_due:
                    time.sleep(delay)
                    continue
            if args.smoke and completed == 60:
                desired = initial + np.array([0.,-.12,0.,0.,-.20,.12,.10,.05])
                head_desired = np.array([.35, .20])
            if not args.smoke:
                section_started = time.monotonic()
                # A full articulation read can synchronize device state. Both
                # controllers use the same fresh snapshot rather than each
                # triggering a separate read of the identical ten joints.
                before_step = robot.get_joint_positions()
                profile_section("joint_read_before", section_started)
                sample_now = time.monotonic()
                sample_wall = node.get_clock().now().nanoseconds*1e-9 if node else time.time()
                desired = np.asarray(setpoints.sample(
                    before_step[indices], sample_now, sample_wall))
                head_desired = np.asarray(head_setpoints.sample(
                    before_step[head_indices], sample_now, sample_wall))
            # Apply one indexed action so a neck update cannot replace the arm
            # controller's targets. Neither live path teleports joint positions.
            section_started = time.monotonic()
            robot.apply_action(ArticulationAction(joint_positions=np.concatenate((desired, head_desired)),
                                                  joint_indices=upper_body_indices))
            profile_section("apply_control", section_started)
            if render_this_step and head_stereo:
                head_stereo.prepare_render(time.monotonic())
            step_started = time.monotonic()
            world.step(render=render_this_step)
            timing.completed(step_started, time.monotonic(), render_this_step)
            profile_section("step_render" if render_this_step else "step_physics", step_started)
            if render_this_step and head_stereo:
                # Metadata only: retain each renderer reference's first observed
                # UTC time so deferred head callbacks cannot relabel old frames.
                head_stereo.observe_render_reference(
                    camera.get_current_frame(clone=False),
                    node.get_clock().now().nanoseconds if node else time.time_ns(),time.monotonic())
            section_started = time.monotonic()
            after_step = robot.get_joint_positions()
            profile_section("joint_read_after", section_started)
            measured = after_step[indices]
            head_measured = after_step[head_indices]
            if not np.all(np.isfinite(measured)) or not np.all(np.isfinite(head_measured)):
                raise RuntimeError("Nonfinite simulated state")
            if first is None:
                first = measured.copy()
            if node and completed % 2 == 0:
                section_started = time.monotonic()
                position, quaternion = terminal.get_world_pose()
                tip = position + quat_to_rot_matrix(quaternion) @ tool_offset
                profile_section("terminal_pose", section_started)
                stamp = node.get_clock().now().to_msg()
                state = JointState()
                state.header.frame_id, state.header.stamp = "base_link", stamp
                state.name = list(limits)
                actual = dict(zip(BOTH_ARM_JOINTS, measured))
                actual.update(zip(HEAD_JOINTS, head_measured))
                state.position = [float(actual.get(name, 0.0)) for name in state.name]
                state_pub.publish(state)
                publish_pose(ee_pub, tip, quaternion, stamp)
            if node and time.monotonic() >= next_head_status:
                next_head_status = time.monotonic()+.1
                head_status = head_setpoints.status(
                    head_measured, time.monotonic(), node.get_clock().now().nanoseconds*1e-9)
                if args.smoke:
                    head_status.update(active=False, reason="simulation_smoke", limited=False,
                                       targets=head_desired.tolist())
                head_status_pub.publish(String(data=json.dumps(head_status, allow_nan=False)))
            if time.monotonic() >= next_timing_diagnostic:
                next_timing_diagnostic = time.monotonic()+5.
                timing_status = timing.diagnostic(time.monotonic(), float(world.current_time))
                timing_status.update(mode="legacy" if args.smoke or args.legacy_loop_timing else "wall_time_catchup",
                                     head_command_age_s=head_setpoints.command_age(time.monotonic(),
                                         node.get_clock().now().nanoseconds*1e-9 if node else time.time()),
                                     head_source_to_receipt_s=head_setpoints.source_to_receipt_s,
                                     sections={name:dict(calls=count, total_ms=total*1000.,
                                                        mean_ms=total/count*1000., max_ms=maximum*1000.)
                                               for name,(total,count,maximum) in profile_sections.items()})
                print("DEICTIC_K1_TIMING "+json.dumps(timing_status, allow_nan=False), flush=True)
                profile_sections.clear()
            # Inspect only after existing render calls, retaining the exact
            # same-frame/pose guards. The cap uses wall time, never simulated
            # time or a loop-counter assumption about physics substeps.
            camera_processing_started = time.monotonic()
            if render_this_step and time.monotonic() >= next_camera_publication:
                wrist_frame = snapshot(camera)
                head_frame = snapshot(headset,require_depth=True) if headset else None
                valid = wrist_frame is not None and (not headset or synchronized(wrist_frame,head_frame))
                if valid and wrist_frame.reference != last_camera_reference:
                    next_camera_publication = time.monotonic()+1/15
                    last_camera_reference = wrist_frame.reference
                    rendered = wrist_frame.rgb
                    cam_pos = wrist_frame.base_from_optical[:3,3]
                    cam_quat = rot_matrix_to_quat(wrist_frame.base_from_optical[:3,:3])
                    frame_count += 1
                    if headset and not pair_captured and frame_count >= 20:
                        pair_captured = capture_pair(args.output,camera,headset,(wrist_frame,head_frame))
                        if pair_captured:
                            print("DEICTIC_CAMERA_PAIR_SAVED",flush=True)
                    if node:
                        stamp = node.get_clock().now().to_msg()
                        publish_pose(camera_pose_pub, cam_pos, cam_quat, stamp)
                        image = Image()
                        image.header.frame_id, image.header.stamp = "k1_wrist_camera_optical", stamp
                        image.height, image.width = rendered.shape[:2]
                        image.encoding, image.step = "rgb8", image.width*3
                        image.data = rendered.tobytes()
                        image_pub.publish(image)
                        info = CameraInfo()
                        info.header, info.width, info.height = image.header, image.width, image.height
                        intrinsic = intrinsics(camera)
                        info.k = intrinsic.reshape(-1).astype(float).tolist()
                        info.r = np.eye(3).reshape(-1).tolist()
                        projection = np.zeros((3,4)); projection[:,:3] = intrinsic
                        info.p = projection.reshape(-1).tolist()
                        info.distortion_model, info.d = "plumb_bob", [0.0]*5
                        info_pub.publish(info)
                        if headset:
                            if head_frame is not None:
                                head_image = Image()
                                head_image.header.frame_id, head_image.header.stamp = "headset_camera_optical", stamp
                                head_image.height, head_image.width = head_frame.rgb.shape[:2]
                                head_image.encoding, head_image.step = "rgb8",head_image.width*3
                                head_image.data = head_frame.rgb.tobytes()
                                head_image_pub.publish(head_image)
                                depth_image = Image()
                                depth_image.header = head_image.header
                                depth_image.height, depth_image.width = head_image.height,head_image.width
                                depth_image.encoding, depth_image.step = "32FC1",head_image.width*4
                                depth_image.data = np.ascontiguousarray(head_frame.depth,dtype="<f4").tobytes()
                                head_depth_pub.publish(depth_image)
                                head_info = CameraInfo()
                                head_info.header, head_info.width, head_info.height = head_image.header,head_image.width,head_image.height
                                head_k = intrinsics(headset)
                                head_info.k = head_k.reshape(-1).astype(float).tolist()
                                head_info.r = np.eye(3).reshape(-1).tolist()
                                head_p = np.zeros((3,4)); head_p[:,:3] = head_k
                                head_info.p = head_p.reshape(-1).tolist()
                                head_info.distortion_model, head_info.d = "plumb_bob",[0.]*5
                                head_info_pub.publish(head_info)
                                head_pos = head_frame.base_from_optical[:3,3]
                                head_quat = rot_matrix_to_quat(head_frame.base_from_optical[:3,:3])
                                publish_pose(head_pose_pub, head_pos+np.array([1.2,0.,.9]),head_quat,stamp,"headset_world")
            if render_this_step and head_stereo:
                head_stereo.finish_render(time.monotonic())
            if render_this_step:
                profile_section("camera_processing", camera_processing_started)
            completed += 1
            if args.legacy_loop_timing and not args.smoke:
                time.sleep(max(0., 1/120-(time.monotonic()-begin)))
        position, quaternion = terminal.get_world_pose()
        tip = position + quat_to_rot_matrix(quaternion) @ tool_offset
        summary = {"steps": completed, "joint_names": list(BOTH_ARM_JOINTS), "initial": first.tolist(),
                   "measured": measured.tolist(), "desired": desired.tolist(), "rendered_frames": frame_count,
                   "head_joint_names": list(HEAD_JOINTS), "head_measured": head_measured.tolist(),
                   "head_desired": head_desired.tolist(),
                   "head_max_tracking_error_rad": float(np.max(np.abs(head_measured-head_desired))),
                   "timing": timing.diagnostic(time.monotonic(), float(world.current_time)),
                   "tip_position_base": tip.tolist(), "camera_position_base": cam_pos.tolist() if frame_count else None,
                   "max_tracking_error_rad": float(np.max(np.abs(measured-desired)))}
        if rendered is not None:
            from PIL import Image as PILImage
            PILImage.fromarray(rendered).save(args.output/"wrist_camera.png")
            summary["image_std"] = float(rendered.std())
            if headset:
                capture_pair(args.output,camera,headset)
        (args.output/"last_run.json").write_text(json.dumps(summary, indent=2))
        print("DEICTIC_K1_RESULT " + json.dumps(summary), flush=True)
        if args.smoke and (completed < 120 or summary["max_tracking_error_rad"] > .06 or frame_count == 0
                           or summary["head_max_tracking_error_rad"] > .06 or np.linalg.norm(head_measured) < .1
                           or np.linalg.norm(measured[:4]-first[:4]) < .05
                           or np.linalg.norm(measured[4:]-first[4:]) < .1 or summary.get("image_std",0) < 1):
            raise RuntimeError("K1 smoke test failed: inspect last_run.json")
    finally:
        if executor:
            executor.shutdown()
        if node:
            node.destroy_node()
            rclpy.shutdown()


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    print("DEICTIC_K1_FAILED", flush=True)
    raise
finally:
    app.close()
