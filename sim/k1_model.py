"""Pinned K1 model metadata and pure-Python simulator command validation."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "models/K1/K1_22dof.urdf"
ARM_JOINTS = (
    "aaright_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_elbow_pitch_joint", "right_elbow_yaw_joint",
)
LEFT_ARM_JOINTS = (
    "aaleft_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint",
)
BOTH_ARM_JOINTS = LEFT_ARM_JOINTS + ARM_JOINTS
TOOL_OFFSET = (0.0, -0.10, 0.0)


def joint_limits(path=URDF):
    return {
        joint.attrib["name"]: (float(joint.find("limit").attrib["lower"]),
                                float(joint.find("limit").attrib["upper"]))
        for joint in ET.parse(path).getroot().findall("joint")
        if joint.attrib["type"] == "revolute"
    }


def validate_positions(names, positions, limits=None, joint_order=ARM_JOINTS):
    """Reject malformed/out-of-range commands rather than silently clipping them."""
    if len(names) != len(joint_order) or set(names) != set(joint_order) or len(positions) != len(joint_order):
        raise ValueError(f"A command must contain each of the {len(joint_order)} required arm joints exactly once")
    limits = limits or joint_limits()
    values = dict(zip(names, positions))
    for name, value in values.items():
        low, high = limits[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"Invalid position for {name}: {value}")
    return [values[name] for name in joint_order]


def merge_arm_positions(names, positions, current, limits=None):
    """Accept legacy right-only or complete bimanual targets, preserving the left hold."""
    if len(current) != len(BOTH_ARM_JOINTS) or not all(math.isfinite(v) for v in current):
        raise ValueError("Eight finite current arm positions are required")
    if len(names) == len(ARM_JOINTS):
        return list(current[:4]) + validate_positions(names, positions, limits)
    return validate_positions(names, positions, limits, BOTH_ARM_JOINTS)


def fixed_arm_urdf(destination, source=URDF):
    """Derive an import-only model: both four-joint arms move; trunk/legs/head stay fixed."""
    tree = ET.parse(source)
    for joint in tree.getroot().findall("joint"):
        if joint.attrib["name"] not in BOTH_ARM_JOINTS:
            joint.set("type", "fixed")
    for mesh in tree.getroot().iter("mesh"):
        mesh.set("filename", str((Path(source).parent / mesh.attrib["filename"]).resolve()))
    tree.write(destination, encoding="utf-8", xml_declaration=True)


class Trajectory:
    """Piecewise-linear simulator setpoints, relative to command receipt."""
    def __init__(self, names, points, initial, limits=None):
        if not points:
            raise ValueError("Trajectory contains no points")
        self.times, self.positions = [0.0], [list(initial)]
        previous = -1.0
        for elapsed, positions in points:
            if not math.isfinite(elapsed) or elapsed < 0 or elapsed <= previous:
                raise ValueError("Trajectory times must be finite, nonnegative and strictly increasing")
            ordered = validate_positions(names, positions, limits)
            if elapsed == 0.0:
                # A zero-time sample must agree with the current measured start.
                if max(abs(a-b) for a,b in zip(ordered, initial)) > 0.05:
                    raise ValueError("Zero-time trajectory start is inconsistent with joint state")
                self.positions[0] = ordered
            else:
                self.times.append(elapsed)
                self.positions.append(ordered)
            previous = elapsed

    def sample(self, elapsed):
        for i in range(1, len(self.times)):
            if elapsed < self.times[i]:
                fraction = max(0.0, (elapsed-self.times[i-1])/(self.times[i]-self.times[i-1]))
                return [a+(b-a)*fraction for a,b in zip(self.positions[i-1], self.positions[i])]
        return list(self.positions[-1])
