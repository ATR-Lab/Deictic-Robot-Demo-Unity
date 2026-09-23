#!/usr/bin/env python3
"""Opt-in bounded head tracking and actual stereo-camera test in identified Isaac.

Stop/disconnect Unity first. Synthetic FLU quaternion commands exercise the real
head actuator boundary; this does not verify native headset input or hardware.
The final action is an inactive measured hold, never a return-to-zero command.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

from head_control import orientation_targets
from k1_model import HEAD_JOINTS

NODE_NAME = "synthetic_isaac_head_verifier"
ISAAC_NODE = "k1_fixed_base_isaac"
POSITION_TOLERANCE = .035
HOLD_SPAN_TOLERANCE = .03


def quaternion(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw/2), math.sin(yaw/2), math.cos(pitch/2), math.sin(pitch/2)
    return [-sy*sp, cy*sp, sy*cp, cy*cp]


def graph_ready(graph):
    return (graph["isaac_node_count"] == 1
            and graph["head_consumers"] == [ISAAC_NODE]
            and graph["head_status_publishers"] == [ISAAC_NODE]
            and graph["joint_publishers"] == [ISAAC_NODE]
            and graph["left_publishers"] == [ISAAC_NODE]
            and graph["right_publishers"] == [ISAAC_NODE]
            and graph["head_publishers"] == [NODE_NAME])


def rgb_digest(message):
    """Hash actual packed ROS RGB pixels (exclude transport row padding)."""
    width, height, step = int(message.width), int(message.height), int(message.step)
    stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
    if (message.encoding != "rgb8" or not 0 < width <= 640 or not 0 < height <= 480
            or not width*3 <= step <= 4096 or len(message.data) != step*height
            or message.header.stamp.sec < 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000
            or stamp <= 0 or not message.header.frame_id):
        raise ValueError("Invalid bounded RGB8 camera message")
    data = bytes(message.data)
    packed = b"".join(data[row*step:row*step+width*3] for row in range(height))
    return dict(stamp_ns=stamp, frame_id=message.header.frame_id, width=width, height=height,
                sha256=hashlib.sha256(packed).hexdigest(), minimum=min(packed), maximum=max(packed))


def paired_digests(left, right):
    if (left["stamp_ns"] != right["stamp_ns"] or left["frame_id"] == right["frame_id"]
            or (left["width"], left["height"]) != (right["width"], right["height"])):
        raise ValueError("Stereo pair requires one exact acquisition stamp and distinct same-size eyes")
    return dict(stamp_ns=left["stamp_ns"], left=left, right=right)


def span(samples):
    if len(samples) < 3:
        raise ValueError("Hold requires at least three independent measured samples")
    return [max(q[index] for q in samples)-min(q[index] for q in samples) for index in range(2)]


def prepare_output(path):
    """Prove the evidence file can be opened for writing before any ROS motion."""
    path.parent.mkdir(parents=True, exist_ok=True)
    # Append mode preserves any previous evidence while checking both an
    # existing file's write permission and a new file's parent permissions.
    with path.open("a", encoding="utf-8") as probe:
        probe.write("")
        probe.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-sim-motion", action="store_true")
    parser.add_argument("--output", required=True, type=Path, help="JSON evidence file")
    parser.add_argument("--timeout", type=float, default=120., help="Whole-run bound in seconds, 60..180")
    parser.add_argument("--check-limits", action="store_true",
                        help="Also request yaw1.4/pitch1.0; simulator must clamp to authored joint limits")
    args = parser.parse_args()
    if not args.allow_sim_motion:
        parser.error("Add --allow-sim-motion only for the isolated Isaac simulator")
    if not math.isfinite(args.timeout) or not 60 <= args.timeout <= 180:
        parser.error("Timeout must be 60..180 seconds")
    try:
        prepare_output(args.output)
    except OSError as error:
        parser.error(f"Evidence output is not writable; no simulation commands sent: {error}")
    import rclpy
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String

    report = dict(passed=False, started_unix=time.time(), phases=[], errors=[],
                  scope="Synthetic head ROS commands and actual Isaac camera pixels; no native XR/hardware test",
                  position_tolerance_rad=POSITION_TOLERANCE, hold_span_tolerance_rad=HOLD_SPAN_TOLERANCE,
                  statuses=[], joints=[], pairs=[], commands=[], check_limits=args.check_limits)
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    publisher = node.create_publisher(String, "/k1/head/command", 1)
    latest = dict(status=None, joints=None, pair=None)
    pending = {}
    session, sequence, authorized = str(uuid.uuid4()), 0, False
    deadline = time.monotonic()+args.timeout
    next_publish = 0.

    def wall():
        return node.get_clock().now().nanoseconds*1e-9

    def status_callback(message):
        try:
            value = json.loads(message.data)
            if (value.get("schema_version") != 1 or value.get("names") != list(HEAD_JOINTS)
                    or not -.05 <= wall()-value.get("stamp", 0.) <= .75):
                return
            if latest["status"] and value["stamp"] <= latest["status"]["stamp"]:
                return
            latest["status"] = value
            report["statuses"].append(value)
        except (ValueError, TypeError, AttributeError):
            report["errors"].append("Malformed head status")

    def joint_callback(message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(values[name]) for name in HEAD_JOINTS):
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if not -.05 <= wall()-stamp <= .75 or (latest["joints"] and stamp <= latest["joints"]["stamp"]):
            return
        sample = dict(stamp=stamp, positions=[float(values[name]) for name in HEAD_JOINTS])
        latest["joints"] = sample
        report["joints"].append(sample)

    def image_callback(eye, message):
        try:
            digest = rgb_digest(message)
            stamp = digest["stamp_ns"]
            if not -.05 <= wall()-stamp*1e-9 <= 1.:
                return
            frame = pending.setdefault(stamp, {})
            frame[eye] = digest
            if "left" in frame and "right" in frame:
                pair = paired_digests(frame["left"], frame["right"])
                if latest["pair"] is None or stamp > latest["pair"]["stamp_ns"]:
                    latest["pair"] = pair
                    report["pairs"].append(pair)
            for old in sorted(pending)[:-8]:
                del pending[old]
        except (ValueError, TypeError, AttributeError) as error:
            if len(report["errors"]) < 50:
                report["errors"].append(str(error))

    node.create_subscription(String, "/k1/head/status", status_callback, 10)
    node.create_subscription(JointState, "/joint_states", joint_callback, 10)
    for eye in ("left", "right"):
        node.create_subscription(Image, f"/k1/head_camera/{eye}/image_raw",
                                 lambda message, eye=eye: image_callback(eye, message), 2)

    def graph():
        publishers = lambda topic: sorted(info.node_name for info in node.get_publishers_info_by_topic(topic))
        return dict(isaac_node_count=node.get_node_names().count(ISAAC_NODE),
                    head_consumers=sorted(info.node_name for info in node.get_subscriptions_info_by_topic("/k1/head/command")),
                    head_publishers=publishers("/k1/head/command"),
                    head_status_publishers=publishers("/k1/head/status"),
                    joint_publishers=publishers("/joint_states"),
                    left_publishers=publishers("/k1/head_camera/left/image_raw"),
                    right_publishers=publishers("/k1/head_camera/right/image_raw"))

    def fresh():
        return all(latest[name] is not None and -.05 <= wall()-latest[name]["stamp"] <= .75
                   for name in ("status", "joints"))

    def send(orientation=None, active=False, tracked=True, force=False):
        nonlocal sequence, next_publish
        if not authorized:
            raise ValueError("Read-only preflight did not authorize simulation commands")
        if not force and time.monotonic() < next_publish:
            return
        sequence += 1
        command = dict(schema_version=1, frame_id="base_link", session_id=session,
                       sequence=sequence, stamp=wall(), active=active, tracked=tracked,
                       orientation=orientation or [0., 0., 0., 1.])
        publisher.publish(String(data=json.dumps(command, allow_nan=False)))
        report["commands"].append(command)
        next_publish = time.monotonic()+.05

    def tick():
        if time.monotonic() >= deadline:
            raise TimeoutError("Whole-run head verification deadline exceeded")
        if authorized and not graph_ready(graph()):
            raise ValueError("Isaac ownership/publisher graph changed during test")
        rclpy.spin_once(node, timeout_sec=.02)

    def motion(label, yaw, pitch):
        orientation = quaternion(yaw, pitch)
        targets = orientation_targets(orientation)
        started, converged, matching = wall(), None, []
        phase_deadline = min(deadline, time.monotonic()+18.)
        last_joint_stamp = 0.
        while time.monotonic() < phase_deadline:
            send(orientation, True)
            tick()
            if not fresh():
                continue
            status, measured = latest["status"], latest["joints"]
            accepted = (status.get("session_id") == session and status.get("active") is True
                        and status["stamp"] >= started
                        and max(abs(a-b) for a, b in zip(status.get("targets", []), targets)) < 1e-5)
            reached = max(abs(a-b) for a, b in zip(measured["positions"], targets)) <= POSITION_TOLERANCE
            if accepted and reached and measured["stamp"] > last_joint_stamp:
                matching.append(measured["stamp"])
                last_joint_stamp = measured["stamp"]
                if len(matching) >= 3 and matching[-1]-matching[0] >= .15 and converged is None:
                    converged = wall()
            elif not (accepted and reached):
                matching, converged = [], None
            pair = latest["pair"]
            if (converged is not None and pair is not None and pair["stamp_ns"]*1e-9 >= converged
                    and wall()-pair["stamp_ns"]*1e-9 <= 1.):
                phase = dict(name=label, started_unix=started, converged_unix=converged,
                             requested_yaw_pitch=[yaw, pitch], targets=targets,
                             measured=measured["positions"], status=status, stereo_pair=pair)
                report["phases"].append(phase)
                print("DEICTIC_HEAD_PHASE "+json.dumps(phase), flush=True)
                return phase
        raise TimeoutError(f"Head phase {label} did not converge with a fresh synchronized stereo pair")

    def hold(label, command_kind, expected_reason):
        started = wall()
        phase_deadline = min(deadline, time.monotonic()+5.)
        acknowledged, samples, last_stamp = None, [], 0.
        while time.monotonic() < phase_deadline:
            if command_kind == "untracked":
                send(quaternion(.25, -.1), True, False)
            elif command_kind == "inactive":
                send(quaternion(.25, -.1), False)
            tick()
            if not fresh():
                continue
            status, measured = latest["status"], latest["joints"]
            if (status.get("session_id") == session and status.get("active") is False
                    and status.get("reason") == expected_reason and status["stamp"] >= started):
                if acknowledged is None:
                    acknowledged = wall()
                # Allow the physics drive .2s to settle onto its measured hold.
                if wall()-acknowledged >= .2 and measured["stamp"] > last_stamp:
                    samples.append(measured["positions"])
                    last_stamp = measured["stamp"]
                if wall()-acknowledged >= .8 and len(samples) >= 3:
                    spans = span(samples)
                    if max(spans) > HOLD_SPAN_TOLERANCE:
                        raise ValueError(f"{label} measured hold moved {spans} radians")
                    report["phases"].append(dict(name=label, reason=expected_reason,
                                                  joint_span_rad=spans, samples=len(samples), status=status))
                    return
        raise TimeoutError(f"Head hold {label} was not acknowledged and measured")

    try:
        preflight_deadline = time.monotonic()+15.
        while time.monotonic() < preflight_deadline:
            tick()
            if graph_ready(graph()) and fresh():
                break
        else:
            raise ValueError("Unique identified Isaac graph and fresh head feedback unavailable: "+json.dumps(graph()))
        report["preflight_graph"] = graph()
        authorized = True
        send(active=False, force=True)
        hold("startup_inactive", "inactive", "head_view_inactive")
        phases = [motion("neutral", 0., 0.), motion("left_up", .25, -.1), motion("right_down", -.25, .1)]
        for before, after in zip(phases, phases[1:]):
            for eye in ("left", "right"):
                if before["stereo_pair"][eye]["sha256"] == after["stereo_pair"][eye]["sha256"]:
                    raise ValueError(f"Actual {eye} pixels did not change between measured head orientations")
        hold("source_silence", "silence", "head_pose_expired")
        hold("tracking_lost", "untracked", "head_tracking_lost")
        hold("view_inactive", "inactive", "head_view_inactive")
        if args.check_limits:
            limited = motion("authored_limits", 1.4, 1.)
            if limited["status"].get("limited") is not True:
                raise ValueError("Over-range quaternion was not acknowledged as limited")
        report["passed"] = True
    except Exception as error:
        report["errors"].append(f"{type(error).__name__}: {error}")
    finally:
        if authorized:
            # Bounded best-effort explicit release even after assertion/graph failure.
            release_started = wall()
            release_deadline = time.monotonic()+1.5
            released = False
            while time.monotonic() < release_deadline:
                send(active=False)
                rclpy.spin_once(node, timeout_sec=.02)
                status = latest["status"]
                if (status and status.get("session_id") == session and status.get("active") is False
                        and status.get("reason") == "head_view_inactive" and status["stamp"] >= release_started):
                    released = True
                    break
            report["final_status"] = latest["status"]
            report["final_release_acknowledged"] = released
            if not released:
                report["passed"] = False
                report["errors"].append("Final inactive measured-hold acknowledgement not observed")
        report["ended_unix"] = time.time()
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False))
        node.destroy_node()
        rclpy.shutdown()
    print("DEICTIC_HEAD_VERIFICATION "+json.dumps(dict(passed=report["passed"], output=str(args.output),
                                                       errors=report["errors"])), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
