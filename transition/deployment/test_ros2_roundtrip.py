#!/usr/bin/env python3
"""Bounded real-DDS gateway test, with logical or explicitly selected Isaac backend.

Run on the Jazzy simulation host after sourcing /opt/ros/jazzy/setup.bash. Isaac
selection requires exclusive worker ownership; no K1 SDK is imported or used.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("logical", "isaac"), default="logical")
    parser.add_argument("--output", required=True)
    parser.add_argument("--domain-id", type=int, default=173)
    parser.add_argument("--profile", choices=("point_a", "point_b", "home"), default="point_a")
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 200:
        parser.error("Use an isolated domain from 1 through 200")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_STATIC_PEERS"] = ""
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    for variable in ("ROS_DISCOVERY_SERVER", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        os.environ.pop(variable, None)
    # Keep runtime-generated state separate from the repository and other runs.
    run_dir = Path(tempfile.mkdtemp(prefix="transition-ros-roundtrip-"))
    root = Path(__file__).resolve().parents[1]
    os.environ["PYTHONPATH"] = str(root / "src") + os.pathsep + os.environ.get("PYTHONPATH", "")
    log = open(run_dir / "gateway.log", "w+")
    process = subprocess.Popen([sys.executable, str(root / "deployment/ros2_gateway.py"),
                                "--backend", args.backend, "--journal", str(run_dir / "gateway.sqlite")],
                               env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    rclpy.init()
    node = Node("transition_roundtrip_" + uuid.uuid4().hex[:8])
    events, states = [], []
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    commands = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(String, "/transition/event", lambda m: events.append(json.loads(m.data)), reliable)
    node.create_subscription(String, "/transition/state", lambda m: states.append(json.loads(m.data)), reliable)
    publisher = node.create_publisher(String, "/transition/command", commands)

    def wait_for(predicate, seconds):
        deadline = time.monotonic() + seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError("ROS roundtrip condition timed out")
            if process.poll() is not None:
                raise RuntimeError("Gateway process exited")
            rclpy.spin_once(node, timeout_sec=0.05)

    def publish(value):
        message = String()
        message.data = json.dumps(value, allow_nan=False)
        publisher.publish(message)

    receipt = {"backend": args.backend, "hostname": socket.gethostname(), "python": sys.version,
               "ros_distro": os.environ.get("ROS_DISTRO"), "domain_id": args.domain_id,
               "discovery": "LOCALHOST", "static_peers": "", "rmw": "rmw_fastrtps_cpp", "hardware_connected": False,
               "journal": str(run_dir / "gateway.sqlite")}
    command_id = None
    request = None
    try:
        wait_for(lambda: states and publisher.get_subscription_count() > 0, 15)
        start = time.monotonic()
        command_id, request_id = str(uuid.uuid4()), str(uuid.uuid4())
        label = {"point_a": "A", "point_b": "B", "home": None}[args.profile]
        command = {"command_id": command_id, "run_id": "ros-roundtrip-" + uuid.uuid4().hex,
                   "skill_id": {"point_a": "point-a", "point_b": "point-b", "home": "home"}[args.profile],
                   "kind": "point", "parameters": {"profile": args.profile}, "dependency_versions": {},
                   "authority_id": "isolated-simulator-roundtrip", "authority_revision": 1,
                   "issued_at": start, "deadline": start + args.timeout}
        request = {"schema_version": 1, "request_id": request_id, "operation": "start",
                   "clock_id": states[-1]["clock_id"], "command": command}
        publish(request)
        wait_for(lambda: any(e.get("event", {}).get("status") in {"succeeded", "failed", "unknown"}
                             and e.get("request_id") == request_id for e in events), args.timeout)
        terminal = next(e for e in events if e.get("request_id") == request_id
                        and e.get("event", {}).get("status") in {"succeeded", "failed", "unknown"})
        assert terminal["event"]["status"] == "succeeded", terminal
        assert terminal["event"]["facts"]["remote.pointed_target"]["value"] == label
        if args.backend == "isaac":
            configuration = json.loads((root / "config/isaac_k1.json").read_text())
            evidence = terminal["event"]["evidence"]
            assert evidence["fixed_base"] is True
            assert evidence["settle_clock"] == "advancing_physics_simulation_time"
            assert evidence["velocity_source"] == "position_difference_per_sim_second"
            for metric, limit in (("joint_error_rad", "joint_tolerance_rad"),
                                  ("reference_error_m", "reference_tolerance_m"),
                                  ("max_velocity_rad_s", "velocity_tolerance_rad_s")):
                assert math.isfinite(evidence[metric]) and 0 <= evidence[metric] <= configuration[limit]
            assert evidence["settle_seconds"] >= configuration["settle_seconds"]
            assert set(evidence["measured_joints_rad"]) == set(configuration["joint_names"])
        wait_for(lambda: any(s["observation"]["quiescent"] and
                             s["observation"].get("facts", {}).get("remote.pointed_target", {}).get("value") == label
                             and s["observation"].get("facts", {}).get("remote.pointed_target", {}).get("status") == "known"
                             for s in states), 5)
        status_id = str(uuid.uuid4())
        publish({"schema_version": 1, "request_id": status_id, "operation": "status",
                 "clock_id": request["clock_id"], "command_id": command_id})
        wait_for(lambda: any(e.get("request_id") == status_id for e in events), 5)
        queried = next(e for e in events if e.get("request_id") == status_id)
        assert queried["event"]["status"] == "succeeded"
        # Same command under a new request ID queries the ledger; it cannot move again.
        duplicate_id = str(uuid.uuid4())
        publish({**request, "request_id": duplicate_id})
        wait_for(lambda: any(e.get("request_id") == duplicate_id for e in events), 5)
        duplicate = next(e for e in events if e.get("request_id") == duplicate_id)
        assert duplicate["event"]["status"] == "succeeded"
        assert duplicate["event"]["at"] == terminal["event"]["at"]
        receipt.update(passed=True, elapsed_seconds=time.monotonic() - start, terminal=terminal,
                       queried=queried, duplicate=duplicate, last_state=states[-1],
                       event_count=len(events), state_count=len(states),
                       native_fixed_base_physics_pipeline_observed=args.backend == "isaac",
                       file_sha256={str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                                    for path in (root / "deployment/ros2_gateway.py", Path(__file__),
                                                 root / "deployment/isaac_state.py", root / "deployment/isaac_worker.py",
                                                 root / "src/transition_autonomy/backends/isaac_backend.py",
                                                 root / "config/isaac_k1.json") if path.exists()})
    except Exception as exc:
        receipt.update(passed=False, error=repr(exc), events=events, last_state=states[-1] if states else None)
        raise
    finally:
        if not receipt.get("passed") and command_id and request and process.poll() is None:
            # Stop only this test's command; never a generic worker reset/stop.
            cleanup_id = str(uuid.uuid4())
            publish({"schema_version": 1, "request_id": cleanup_id, "operation": "stop",
                     "clock_id": request["clock_id"], "command_id": command_id})
            cleanup_end = time.monotonic() + 5.0
            while time.monotonic() < cleanup_end:
                rclpy.spin_once(node, timeout_sec=0.05)
                if states and states[-1]["observation"]["active_command_id"] is None:
                    break
            receipt["cleanup_state"] = states[-1] if states else None
            receipt["cleanup_replies"] = [e for e in events if e.get("request_id") == cleanup_id]
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.flush()
        log.seek(0)
        receipt["gateway_log"] = log.read()
        log.close()
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(receipt, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"passed": receipt["passed"], "backend": args.backend,
                      "receipt": args.output, "elapsed_seconds": receipt["elapsed_seconds"]}))


if __name__ == "__main__":
    main()
