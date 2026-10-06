#!/usr/bin/env python3
"""Native Isaac Sim 5.0 K1 physics worker. Run using /isaac-sim/python.sh.

HTTP threads only handle the ledger and snapshots. The main thread exclusively
owns USD, PhysX, articulation commands and rendering.
"""
from __future__ import annotations
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
import time
import urllib.parse
import xml.etree.ElementTree as ET

from isaac_state import WorkerState
from isaac_streaming import configure_streaming, public_ipv4, select_spectator_viewport, SPECTATOR_CAMERA


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/isaac_k1.json")
    parser.add_argument("--assets", default="assets/booster_k1")
    parser.add_argument("--output", default="artifacts/isaac")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--ros", action=argparse.BooleanOptionalAction, default=False,
                        help="Publish measured right-arm state and demanded head-eye images; no ROS control inputs")
    parser.add_argument("--head-stereo-rate", type=float, default=15.)
    parser.add_argument("--webrtc", action="store_true",
                        help="Enable the Isaac Sim 5.0 spectator stream on TCP 49100 and UDP 47998")
    parser.add_argument("--public-ip", default="",
                        help="Literal workstation IPv4 address reachable by the streaming client; required with --webrtc")
    args = parser.parse_args()
    if not 1. <= args.head_stereo_rate <= 30.:
        parser.error("head-stereo-rate must be between 1 and 30 Hz")
    if args.webrtc or args.public_ip:
        try:
            args.public_ip = public_ipv4(args.public_ip)
        except ValueError as exc:
            parser.error(str(exc))
    config = json.loads(Path(args.config).read_text())
    assets = Path(args.assets).resolve()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    vendor_urdf = assets / "K1_22dof.urdf"
    if hashlib.sha256(vendor_urdf.read_bytes()).hexdigest() != config["urdf_sha256"]:
        raise ValueError("K1 vendor URDF hash mismatch")
    tree = ET.parse(vendor_urdf)
    joint_elements = {j.get("name"): j for j in tree.findall("joint")}
    for profile in config["profiles"].values():
        if len(profile["joints_rad"]) != len(config["joint_names"]):
            raise ValueError("profile joint count mismatch")
        for name, value in zip(config["joint_names"], profile["joints_rad"]):
            limit = joint_elements[name].find("limit")
            if not float(limit.get("lower")) <= value <= float(limit.get("upper")):
                raise ValueError("profile violates official joint limit")
    for joint in tree.findall("joint"):
        if joint.get("name") not in config["joint_names"]:
            joint.set("type", "fixed")
    for mesh in tree.iter("mesh"):
        filename = (assets / mesh.get("filename")).resolve()
        if not filename.is_relative_to(assets) or not filename.is_file():
            raise ValueError("missing or invalid official mesh")
        mesh.set("filename", str(filename))
    derived = output / "k1_fixed_support.urdf"
    tree.write(derived, encoding="utf-8", xml_declaration=True)

    from isaacsim import SimulationApp
    app_config = {"headless": True, "width": 960, "height": 720,
                  "renderer": "RayTracedLighting", "anti_aliasing": 0}
    if args.webrtc:
        app_config["hide_ui"] = False  # The streaming extension captures the app viewport.
    app = SimulationApp(app_config)
    # NVIDIA requires imports of simulation/Omniverse modules after SimulationApp.
    import numpy as np
    import omni.kit.commands
    import omni.usd
    import omni.replicator.core as rep
    from pxr import Gf, Usd, UsdGeom, UsdLux
    from PIL import Image
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.rotations import rot_matrix_to_quat
    from isaacsim.sensors.camera import Camera
    from isaacsim.asset.importer.urdf import _urdf
    from isaac_telemetry import SimulationTelemetry, configure_camera, camera_intrinsics

    streaming = configure_streaming(app, enabled=args.webrtc, public_ip=args.public_ip,
                                    enable_extension=enable_extension)

    # Expected geometry comes from the pinned URDF. Observed geometry below comes
    # independently from the simulated terminal-link transform after physics.
    def rotation(axis, angle):
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)
        x,y,z = axis
        cross = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3) + np.sin(angle)*cross + (1-np.cos(angle))*(cross@cross)

    def forward(joints):
        by_child = {j.find("child").get("link"): j for j in joint_elements.values()}
        chain = []
        link = config["terminal_link"]
        while link != "trunk":
            joint = by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        matrix = np.eye(4)
        positions = dict(zip(config["joint_names"], joints))
        for joint in reversed(chain):
            origin = joint.find("origin")
            local = np.eye(4)
            local[:3,3] = [float(x) for x in origin.get("xyz", "0 0 0").split()]
            r,p,y = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            local[:3,:3] = rotation([0,0,1],y) @ rotation([0,1,0],p) @ rotation([1,0,0],r)
            matrix = matrix @ local
            if joint.get("name") in positions:
                move = np.eye(4)
                move[:3,:3] = rotation([float(x) for x in joint.find("axis").get("xyz").split()], positions[joint.get("name")])
                matrix = matrix @ move
        return (matrix @ np.array(config["tool_offset_m"] + [1]))[:3].tolist()

    world = World(stage_units_in_meters=1.0, physics_dt=1/120, rendering_dt=1/30)
    world.scene.add_default_ground_plane(z_position=-.70)
    enable_extension("isaacsim.asset.importer.urdf")
    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.import_inertia_tensor = True
    import_config.convex_decomp = False
    import_config.self_collision = False
    import_config.default_drive_strength = 80.0
    import_config.default_position_drive_damping = 4.0
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    ok, root_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=str(derived),
                                             import_config=import_config, get_articulation_root=True)
    if not ok:
        raise RuntimeError("official K1 URDF import failed")
    robot = world.scene.add(SingleArticulation(root_path, name="k1"))
    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(1500)
    state = WorkerState(config, output / "command_ledger.jsonl", time.monotonic())
    state.reference_targets = {name: forward(profile["joints_rad"]) for name,profile in config["profiles"].items()}
    for name, point in state.reference_targets.items():
        if name == "home":
            continue
        marker = UsdGeom.Sphere.Define(stage, "/World/target_" + name)
        marker.CreateRadiusAttr(.014)
        marker.AddTranslateOp().Set(Gf.Vec3d(*point))
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.05, .8, .25) if name == "point_a" else Gf.Vec3f(.95, .4, .05)])
    # Keep one continuous camera reference for the exact renderer acquisition ID
    # used by the demand-driven head eyes, rather than stamping deferred images
    # as though they were acquired at their eventual publication time.
    eye, look = np.array([1.4, -1.7, .75]), np.array([0., -.03, -.12])
    forward_axis = (look-eye)/np.linalg.norm(look-eye)
    right_axis = np.cross(forward_axis, np.array([0., 0., 1.]))
    right_axis /= np.linalg.norm(right_axis)
    down_axis = np.cross(forward_axis, right_axis)
    camera = Camera(SPECTATOR_CAMERA, name="transition_spectator", frequency=-1,
                    resolution=(960, 720))
    camera.set_world_pose(eye, rot_matrix_to_quat(np.column_stack((right_axis, down_axis, forward_axis))), camera_axes="ros")
    world.reset()
    camera.initialize()
    configure_camera(camera)
    camera.attach_annotator("CameraParams")
    if set(robot.dof_names) != set(config["joint_names"]):
        raise RuntimeError(f"unexpected DOFs: {robot.dof_names}")
    # Isaac 5.0 importer defaults produced zero damping despite the import
    # config fields above. Set and record actual native controller gains.
    robot.get_articulation_controller().set_gains(kps=np.full(len(robot.dof_names), 4000.0),
                                                 kds=np.full(len(robot.dof_names), 126.5),
                                                 save_to_usd=True)
    robot.set_solver_position_iteration_count(32)
    robot.set_solver_velocity_iteration_count(8)
    robot.set_sleep_threshold(0.0)
    indices = np.array([robot.dof_names.index(name) for name in config["joint_names"]], dtype=int)
    target = np.array(config["profiles"]["home"]["joints_rad"])
    robot.set_joint_positions(target, joint_indices=indices)  # Initial reset only.
    robot.set_joint_velocities(np.zeros(len(indices)), joint_indices=indices)
    terminal = next((p for p in stage.Traverse() if p.GetName() == config["terminal_link"]), None)
    if terminal is None:
        raise RuntimeError("terminal link missing after URDF import")
    telemetry, head_stereo = None, None
    if args.ros:
        shared_sim = Path(__file__).resolve().parents[2] / "sim"
        if not (shared_sim / "head_stereo.py").is_file():
            raise RuntimeError("--ros requires the monorepo's shared sim/head_stereo.py and camera_sync.py")
        sys.path.insert(0, str(shared_sim))
        from head_stereo import HeadStereoDisplay
        telemetry = SimulationTelemetry(config["joint_names"], state.boot_id)
        head_stereo = HeadStereoDisplay(stage, output, telemetry.node, configure_camera, camera_intrinsics,
                                        rate_hz=args.head_stereo_rate, urdf=vendor_urdf)
    if args.webrtc:
        from omni.kit.viewport.utility import get_active_viewport
        # A render product alone does not select the app viewport streamed by
        # WebRTC. Bind after all cameras and the initial articulation are ready.
        select_spectator_viewport(get_active_viewport(), streaming)
        app.update()
    provenance = {"isaac_version": "5.0.0", "model_revision": config["model_revision"],
                  "urdf_sha256": config["urdf_sha256"], "joint_names": config["joint_names"],
                  "articulation_path": str(root_path), "terminal_link_path": str(terminal.GetPath()),
                  "reference_targets_m": state.reference_targets, "fixed_base": True,
                  "all_non_right_arm_joints_fixed": True, "self_collision": False,
                  "drive_gains": {"kp": 4000.0, "kd": 126.5, "type": "acceleration"},
                  "solver_iterations": {"position": 32, "velocity": 8},
                  "articulation_sleep_threshold": 0.0,
                  "velocity_gate_source": "consecutive measured joint positions / positive simulation-time increment",
                  "tool_reference": "virtual point 0.10 m along terminal link negative Y; not fingertip calibration",
                  "boot_id": state.boot_id}
    provenance["ros_visualization"] = {"enabled": args.ros, "measured_joint_names": list(config["joint_names"]),
        "command_subscriptions": [], "head_articulation_fixed": True,
        "stereo_camera": head_stereo.spec if head_stereo else None}
    provenance["webrtc"] = streaming
    (output / "scene_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    stage.GetRootLayer().Export(str(output / "scene.usda"))
    lock = threading.RLock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, payload, code=200):
            payload.update(server_now=time.monotonic(), boot_id=state.boot_id)
            raw = json.dumps(payload, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            with lock:
                if self.path == "/observation":
                    self.respond({"observation": state.snapshot})
                elif self.path == "/events":
                    # Non-destructive reads: caller adapter de-duplicates events.
                    self.respond({"events": state.events[-256:]})
                elif self.path.startswith("/commands/status?"):
                    command_id = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get("id", [""])[0]
                    self.respond({"event": state.ledger.get(command_id, {}).get("event")})
                elif self.path == "/provenance":
                    self.respond({"provenance": provenance})
                else:
                    self.respond({"error": "unknown route"}, 404)

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", 0))
                if not 0 <= length <= 65536:
                    raise ValueError("request too large")
                data = json.loads(self.rfile.read(length))
                with lock:
                    if self.path == "/commands/start":
                        event = state.accept(data["command"], data["remaining_seconds"], time.monotonic())
                        self.respond({"event": event})
                    elif self.path == "/stop":
                        state.stop_requested = True
                        self.respond({"stop_requested": True})
                    else:
                        self.respond({"error": "unknown route"}, 404)
            except (ValueError, KeyError, TypeError) as exc:
                self.respond({"error": str(exc)}, 400)

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    started = time.monotonic()
    step = 0
    last_frame = 0.0
    previous_joints = None
    previous_sim_time = None
    print(json.dumps({"ready": True, "port": args.port, "boot_id": state.boot_id, "provenance": provenance}), flush=True)
    try:
        while app.is_running() and (not args.duration or time.monotonic() - started < args.duration):
            cycle = time.monotonic()
            if telemetry:
                telemetry.spin()
            joints = robot.get_joint_positions(joint_indices=indices).tolist()
            with lock:
                state.begin(joints, cycle)
                desired = state.desired(joints, cycle)
            if desired is not None:
                target = np.array(desired)
            robot.apply_action(ArticulationAction(joint_positions=target, joint_indices=indices))
            render_this_step = step % 4 == 0
            if render_this_step and head_stereo:
                head_stereo.prepare_render(time.monotonic())
            world.step(render=render_this_step)
            step += 1
            now = time.monotonic()
            if render_this_step and head_stereo:
                head_stereo.observe_render_reference(camera.get_current_frame(clone=False),
                    telemetry.node.get_clock().now().nanoseconds, now)
            joints = robot.get_joint_positions(joint_indices=indices).tolist()
            velocities = robot.get_joint_velocities(joint_indices=indices).tolist()
            sim_time = float(world.current_time)
            positional_velocities = None
            if previous_joints is not None and sim_time > previous_sim_time:
                positional_velocities = [(a-b)/(sim_time-previous_sim_time) for a,b in zip(joints,previous_joints)]
            previous_joints, previous_sim_time = joints, sim_time
            transform = UsdGeom.Xformable(terminal).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            reference = list(transform.Transform(Gf.Vec3d(*config["tool_offset_m"])))
            with lock:
                state.sample(joints, positional_velocities if positional_velocities is not None else velocities,
                             reference, now, sim_time, velocity_source="position_difference_per_sim_second" if positional_velocities is not None else "native_initial_sample",
                             native_velocities=velocities)
                state.snapshot["measurements"]["position_difference_velocity_rad_s"] = positional_velocities
                snapshot = dict(state.snapshot)
            if telemetry:
                telemetry.publish(joints, positional_velocities, velocities, now, sim_time)
            if render_this_step and head_stereo:
                head_stereo.finish_render(time.monotonic())
            if now - last_frame >= 2 and render_this_step:
                pixels = camera.get_current_frame(clone=False).get("rgb")
                if pixels is not None and pixels.size:
                    Image.fromarray(pixels).save(output / "latest_frame.png")
                    (output / "latest_observation.json").write_text(json.dumps(snapshot, indent=2) + "\n")
                    last_frame = now
            time.sleep(max(0, 1/120 - (time.monotonic() - cycle)))
    finally:
        server.shutdown()
        server.server_close()
        if telemetry:
            telemetry.close()
        app.close()


if __name__ == "__main__":
    main()
