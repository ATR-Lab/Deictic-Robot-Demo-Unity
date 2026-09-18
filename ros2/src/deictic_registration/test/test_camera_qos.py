"""Camera transport contracts without creating nodes or changing DDS settings."""
import pytest

pytest.importorskip("rclpy")
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy, qos_profile_sensor_data
from deictic_registration.camera_qos import camera_subscription_qos


def test_default_preserves_sensor_data_policy():
    actual = camera_subscription_qos()
    assert actual is qos_profile_sensor_data
    assert actual.reliability == ReliabilityPolicy.BEST_EFFORT


def test_reliable_policy_has_shallow_volatile_history():
    actual = camera_subscription_qos("reliable_latest")
    assert actual.history == HistoryPolicy.KEEP_LAST
    assert actual.depth == 2
    assert actual.reliability == ReliabilityPolicy.RELIABLE
    assert actual.durability == DurabilityPolicy.VOLATILE
    # Selecting reliable must not mutate the shared default profile.
    assert qos_profile_sensor_data.reliability == ReliabilityPolicy.BEST_EFFORT


@pytest.mark.parametrize("invalid", ["", "reliable", None, True, 2])
def test_unknown_policy_is_rejected(invalid):
    with pytest.raises(ValueError, match="camera_qos"):
        camera_subscription_qos(invalid)
