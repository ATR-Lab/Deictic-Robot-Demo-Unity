"""Publish only measured DOFs and preserve source/timestamp provenance."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

path = Path(__file__).resolve().parents[1] / "deployment/isaac_telemetry.py"
spec = importlib.util.spec_from_file_location("isaac_telemetry", path)
telemetry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(telemetry)

NAMES = ["aaright_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_elbow_pitch_joint", "right_elbow_yaw_joint"]


def payload(**kwargs):
    arguments = dict(names=NAMES, joints=[0., .2, .3, .4], derived_velocities=[.1]*4,
                     native_velocities=[7.]*4, boot_id="test-boot", sequence=1,
                     sim_time=1., received_monotonic=100., stamp_ns=1_000_000_000)
    arguments.update(kwargs)
    return telemetry.measurement_payload(**arguments)


def test_payload_distinguishes_measurement_from_fixed_model_and_native_velocity():
    result = payload()
    assert result["joint_names"] == NAMES
    assert len(result["positions_rad"]) == 4
    assert result["fixed_model_parts_are_measured"] is False
    assert result["position_difference_velocity_rad_s"] == [.1]*4
    assert result["native_velocity_rad_s"] == [7.]*4
    assert result["ros_command_inputs"] == [] and result["simulation_only"]


@pytest.mark.parametrize("change", [
    {"joints": [0., 1.]}, {"names": ["a"]*4}, {"joints": [float("nan")]*4},
    {"derived_velocities": [0., float("inf"), 0., 0.]}, {"sim_time": float("inf")},
    {"stamp_ns": 0}, {"sequence": True}, {"joints": [True]*4},
])
def test_invalid_measurement_cannot_publish(change):
    with pytest.raises(ValueError):
        payload(**change)


def test_publisher_does_not_relabel_duplicate_physics_samples():
    publisher = telemetry.SimulationTelemetry.__new__(telemetry.SimulationTelemetry)
    publisher.last_source_time, publisher.next_publish = 1., 0.
    # No ROS node is installed: duplicates/regressions must return before using it.
    assert publisher.publish([0.]*4, None, [0.]*4, 10., 1.) is False
    assert publisher.publish([0.]*4, None, [0.]*4, 10., .5) is False


def test_initial_velocity_is_absent_instead_of_mislabelled_as_derived():
    result = payload(derived_velocities=None)
    assert result["position_difference_velocity_rad_s"] is None
    assert result["joint_state_velocity_source"] == "unavailable_initial_sample"
