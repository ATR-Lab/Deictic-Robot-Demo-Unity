"""Pure validation/conversion for diagnostic K1 telemetry (Python 3.10+).

Source ROS stamps are retained, but their acquisition meaning and clock offset
are uncommissioned. These values must never certify physical task admission.
"""
from __future__ import annotations

import math

# Application-side retention/decoder bounds. ROS/DDS has already received the
# message before these checks run; these are not middleware memory limits.
MAX_RAW_IMAGE_BYTES = 6 * 1024 * 1024
MAX_ROW_PADDING_BYTES = 256
MAX_FRAME_ID_BYTES = 256
RAW_CAMERA_TOPICS = (
    ("left", "/boostercamera/head/raw/rgb"),
    ("right", "/boostercamera/head/raw/right/rgb"),
)

VENDOR_NAMES = (
    "AAHead_Yaw", "Head_Pitch", "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll",
    "Left_Elbow_Pitch", "Left_Elbow_Yaw", "ARight_Shoulder_Pitch", "Right_Shoulder_Roll",
    "Right_Elbow_Pitch", "Right_Elbow_Yaw", "Left_Hip_Pitch", "Left_Hip_Roll",
    "Left_Hip_Yaw", "Left_Knee_Pitch", "Left_Ankle_Pitch", "Left_Ankle_Roll",
    "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch",
    "Right_Ankle_Pitch", "Right_Ankle_Roll",
)
URDF_NAMES = (
    "aahead_yaw_joint", "aahead_pitch_joint", "aaleft_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint", "aaright_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_elbow_pitch_joint", "right_elbow_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
    "left_hip_yaw_joint", "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_pitch_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
)


def mapped_joints(names, positions, velocities):
    """Map by explicit source names, never positional assumptions or sorting."""
    if len(names) != 22 or len(set(names)) != 22 or set(names) != set(VENDOR_NAMES):
        raise ValueError("Unexpected K1 joint-name set")
    if len(positions) != 22 or len(velocities) not in (0, 22):
        raise ValueError("Invalid joint vector length")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in (*positions, *velocities)):
        raise ValueError("Nonfinite joint telemetry")
    indices = [names.index(name) for name in VENDOR_NAMES]
    return list(URDF_NAMES), [positions[i] for i in indices], [velocities[i] for i in indices] if velocities else []


def stamp_ns(header):
    s = header.stamp
    if s.sec < 0 or not 0 <= s.nanosec < 1_000_000_000:
        raise ValueError("Invalid ROS source stamp")
    value = s.sec * 1_000_000_000 + s.nanosec
    if value <= 0:
        raise ValueError("Missing ROS source stamp")
    return value


def checked_frame_id(header):
    frame = header.frame_id
    if (not isinstance(frame, str) or len(frame) > MAX_FRAME_ID_BYTES or not frame.strip()
            or len(frame.encode("utf-8")) > MAX_FRAME_ID_BYTES
            or any(ord(char) < 32 or ord(char) == 127 for char in frame)):
        raise ValueError("Invalid or oversized camera frame ID")
    return frame


def raw_camera_sources(mono_source, stereo=False):
    """Subscribe only to explicitly requested sources; stereo is opt-in."""
    if mono_source not in ("left", "native") or not isinstance(stereo, bool):
        raise ValueError("Invalid camera source selection")
    return RAW_CAMERA_TOPICS if stereo else RAW_CAMERA_TOPICS[:1] if mono_source == "left" else ()


def checked_raw_image(message):
    """Validate before retaining or copying raw bytes; return canonical layout."""
    checked_frame_id(message.header)
    stamp_ns(message.header)
    w, h, step = message.width, message.height, message.step
    if (any(isinstance(value, bool) or not isinstance(value, int) for value in (w, h, step))
            or not (0 < w <= 2048 and 0 < h <= 2048 and w*h <= 2_097_152)):
        raise ValueError("Camera dimensions exceed diagnostic limits")
    # Check the payload bound before decoder imports, copies, or reshape.
    if not 0 < len(message.data) <= MAX_RAW_IMAGE_BYTES:
        raise ValueError("Raw camera payload exceeds diagnostic byte limit")
    encoding = message.encoding
    if not isinstance(encoding, str) or len(encoding) > 16:
        raise ValueError("Unsupported camera encoding")
    encoding = encoding.lower()
    if encoding == "nv12":
        if (w % 2 or h % 2 or step % 2 or not w <= step <= w + MAX_ROW_PADDING_BYTES
                or len(message.data) != step * h * 3 // 2):
            raise ValueError("Malformed NV12 image")
    elif (encoding not in ("rgb8", "bgr8")
            or not w*3 <= step <= w*3 + MAX_ROW_PADDING_BYTES
            or len(message.data) != step*h):
        raise ValueError("Unsupported camera encoding or stride")
    return w, h, step, encoding


def camera_bgr(message):
    """Bounded NV12/packed-RGB conversion; no inferred intrinsics or rectification."""
    w, h, step, encoding = checked_raw_image(message)
    import cv2
    import numpy as np
    if encoding == "nv12":
        raw = np.frombuffer(message.data, np.uint8).reshape(h*3//2, step)[:, :w].copy()
        return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_NV12)
    raw = np.frombuffer(message.data, np.uint8).reshape(h, step)[:, :w*3].reshape(h,w,3)
    return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else raw.copy()


def pair_metadata(left, right, max_skew_ns=40_000_000):
    a, b = stamp_ns(left.header), stamp_ns(right.header)
    if (left.width, left.height) != (right.width, right.height):
        raise ValueError("Stereo dimensions differ")
    if checked_frame_id(left.header) == checked_frame_id(right.header):
        raise ValueError("Distinct optical frames required")
    if abs(a-b) > max_skew_ns:
        raise ValueError("Stereo source stamp skew exceeds display bound")
    return {"left_source_stamp_ns": a, "right_source_stamp_ns": b, "source_stamp_skew_ns": abs(a-b),
            "left_frame": left.header.frame_id, "right_frame": right.header.frame_id,
            "acquisition_synchronization_verified": False, "rectified": False,
            "use": "diagnostic_display_only"}


def pair_advances(key, previous):
    return previous is None or all(current > old for current, old in zip(key, previous))


def display_size(width, height):
    """Downsample within one-eye 480x360 bounds, without arbitrary aspect change."""
    if width <= 0 or height <= 0:
        raise ValueError('Invalid image size')
    scale = min(1., 480/width, 360/height)
    return max(1, int(round(width*scale))), max(1, int(round(height*scale)))


def checked_jpeg(message):
    """Bound allocation before decoding; preserve original pixels and stamp."""
    if message.format not in ('jpeg', 'rgb8; jpeg compressed bgr8'):
        raise ValueError('Unsupported JPEG format')
    if not 4 <= len(message.data) <= 512*1024:
        raise ValueError('Invalid/bounded JPEG payload')
    data = bytes(message.data)
    if data[:2] != b'\xff\xd8' or data[-2:] != b'\xff\xd9':
        raise ValueError('Invalid/bounded JPEG payload')
    cursor = 2
    dimensions = None
    while cursor < len(data)-2:
        if data[cursor] != 255:
            raise ValueError('Malformed JPEG marker')
        while cursor < len(data) and data[cursor] == 255:
            cursor += 1
        if cursor >= len(data):
            raise ValueError('Truncated JPEG')
        marker = data[cursor]; cursor += 1
        if marker == 0xda:  # Entropy scan follows a checked bounded frame header.
            break
        if cursor+2 > len(data):
            raise ValueError('Truncated JPEG segment')
        size = int.from_bytes(data[cursor:cursor+2], 'big')
        if size < 2 or cursor+size > len(data):
            raise ValueError('Invalid JPEG segment length')
        if marker in (0xc0, 0xc2):
            if dimensions is not None or size < 8 or data[cursor+2] != 8 or data[cursor+7] != 3:
                raise ValueError('Unsupported JPEG frame')
            height = int.from_bytes(data[cursor+3:cursor+5], 'big')
            width = int.from_bytes(data[cursor+5:cursor+7], 'big')
            if not 0 < width <= 960 or not 0 < height <= 360:
                raise ValueError('JPEG dimensions exceed viewer bounds')
            dimensions = (width, height)
        elif 0xc0 <= marker <= 0xcf and marker not in (0xc4, 0xc8, 0xcc):
            raise ValueError('Unsupported JPEG coding')
        cursor += size
    if dimensions is None:
        raise ValueError('JPEG dimensions missing')
    return dimensions


def transcode_mono(message):
    """Small diagnostic JPEG for the robot-to-workstation link; no new stamp."""
    import cv2
    import numpy as np
    checked_jpeg(message)  # Bound dimensions before native decoder allocation.
    bgr = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('Native JPEG decode failed')
    height, width = bgr.shape[:2]
    scale = min(1., 432/width, 360/height)
    size = (max(1, round(width*scale)), max(1, round(height*scale)))
    bgr = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 65])
    if not ok or len(encoded) > 512*1024:
        raise ValueError('Native JPEG transcode failed/exceeded output bound')
    return encoded.tobytes()
