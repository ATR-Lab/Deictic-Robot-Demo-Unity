"""Explicit camera transport choices; estimator freshness is enforced separately."""
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)


def camera_subscription_qos(mode="sensor_data"):
    if mode == "sensor_data":
        return qos_profile_sensor_data
    if mode == "reliable_latest":
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    raise ValueError("camera_qos must be 'sensor_data' or 'reliable_latest'")
