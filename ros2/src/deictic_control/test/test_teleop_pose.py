"""Actual K1 geometry, natural-size pose requests and bounded feedback servo."""
import json
import time
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from deictic_control.kinematics import ArmModel
from deictic_control.teleop import BimanualClutch, ClutchInput, PoseIK, chain_reach


@pytest.fixture
def servo():
    urdf = Path(__file__).resolve().parents[4]/'models/K1/K1_22dof.urdf'
    models = (ArmModel(urdf, tip='left_elbow_yaw_link', tool_offset=[0, .1, 0]), ArmModel(urdf))
    return BimanualClutch(models)


class Driver:
    def __init__(self, servo):
        self.servo, self.sequence = servo, 0
        self.q = np.array([[0., 0., 0., 0.], [0., .5, 0., 0.]])
        self.position, self.quat = np.zeros((2, 3)), np.tile([0., 0., 0., 1.], (2, 1))
        self.send(False); self.send(True)
        self.previous_velocity = np.zeros((2, 4))
        self.latencies = []

    def send(self, held=True):
        self.sequence += 1
        self.now = 100.+self.sequence*.05
        payload = dict(schema_version=2, frame_id='teleop_head', session_id='pose_test',
            sequence=self.sequence, stamp=self.now, clutch=held, left_tracked=True, right_tracked=True,
            left_position=self.position[0].tolist(), right_position=self.position[1].tolist(),
            left_rotation=self.quat[0].tolist(), right_rotation=self.quat[1].tolist())
        return self.servo.ingest(json.dumps(payload), self.now, self.now, self.q)

    def step(self, feedback=True, bounds=True):
        self.send()
        before = self.servo.command.copy()
        started = time.perf_counter()
        command = self.servo.tick(self.now, self.now, self.q)
        self.latencies.append(time.perf_counter()-started)
        assert command is not None, self.servo.reason
        velocity = (command-before)/.05
        assert np.max(np.abs(velocity)) <= self.servo.speed+1e-9
        if bounds:
            assert np.max(np.abs(velocity-self.previous_velocity))/.05 <= self.servo.acceleration+1e-8
        assert np.max(np.abs(command-self.q)) <= .075+1e-9
        for model, q0, q1 in zip(self.servo.models, before, command):
            assert np.linalg.norm(model.fk(q1)[:3, 3]-model.fk(q0)[:3, 3]) <= .005+1e-9
        self.previous_velocity = velocity
        if feedback:
            self.q = command
        return command


def test_v1_explicitly_fails_and_rotations_require_unit_finite_quaternions():
    with pytest.raises(ValueError, match='protocol_version_mismatch'):
        ClutchInput.parse(json.dumps(dict(schema_version=1)))
    base = dict(schema_version=2, frame_id='teleop_head', session_id='a', sequence=1,
        stamp=100., clutch=False, left_tracked=True, right_tracked=True,
        left_position=[0,0,0], right_position=[0,0,0], left_rotation=[0,0,0,1], right_rotation=[0,0,0,1])
    for bad in ([0,0,0,0], [0,0,0,2], [0,0,float('nan'),1], [0,0,1]):
        with pytest.raises((ValueError, TypeError)):
            ClutchInput.parse(json.dumps(dict(base, left_rotation=bad)))


def test_arm_scale_excludes_shoulder_mount_and_small_neutral_noise_never_moves(servo):
    assert chain_reach(servo.models[0])/.60 == pytest.approx(.558833641)
    driver = Driver(servo)
    driver.position[:] = [3e-7, -2e-7, 1e-7]
    driver.quat[:] = Rotation.from_rotvec([1e-6,0,0]).as_quat()
    for _ in range(5):
        np.testing.assert_array_equal(driver.step(), driver.q)


@pytest.mark.parametrize('axis,sign', [(0,-1),(0,1),(1,-1),(1,1),(2,-1),(2,1)])
def test_both_arms_six_axis_5_to_15cm_requests_are_bounded_and_report_saturation(servo, axis, sign):
    driver = Driver(servo)
    # Ramp through 5, 10 and 15 cm; not every Cartesian point is physically
    # reachable from a fully extended arm. No fake convergence claim is made.
    for i in range(100):
        driver.position[:, axis] = sign*.15*min(1., (i+1)/60)
        driver.step()
    assert servo.active and servo.armed
    assert servo.state in ('active', 'limited')
    assert np.all(np.isfinite(servo.measured_position_errors))
    assert max(driver.latencies) < .10, 'One bounded tick must not consume multiple input leases'


def test_straight_left_and_near_singular_right_inward10cm_converge(servo):
    driver = Driver(servo)
    for i in range(220):
        driver.position[:, 1] = np.array([-.10, .10])*min(1., (i+1)/40)
        driver.step()
    assert max(servo.measured_position_errors) < .003
    assert np.max(np.abs(driver.q[0])) > .5, 'Must actually escape the rank-two straight-arm start'
    assert np.percentile(driver.latencies, 95) < .05
    print(json.dumps(dict(benchmark='bilateral_inward10cm_20Hz', samples=len(driver.latencies),
        mean_ms=1000*np.mean(driver.latencies), p95_ms=1000*np.percentile(driver.latencies, 95),
        max_ms=1000*max(driver.latencies), final_position_errors=servo.measured_position_errors)))


def test_measured_lag_does_not_accumulate_commands_and_recovers(servo):
    driver = Driver(servo)
    driver.position[:, 0] = .08
    for _ in range(100):
        driver.step(feedback=False)
    assert servo.active and servo.last_fault == ''
    lag_error = max(servo.measured_position_errors)
    for _ in range(120):
        driver.step()
    assert max(servo.measured_position_errors) < lag_error*.4


def test_saturation_returns_without_a_release_and_pure_orientation_moves(servo):
    driver = Driver(servo)
    driver.position[:, 1] = [.5, -.5]
    for _ in range(30): driver.step()
    assert servo.active and servo.state == 'limited'
    driver.position[:] = 0.
    driver.position[:, 0] = .025
    for _ in range(120): driver.step()
    assert max(servo.measured_position_errors) < .004 and servo.active
    # Reclutch at measured pose, then turn only the controller orientation.
    driver.send(False); driver.send(True)
    driver.previous_velocity[:] = 0.
    target = Rotation.from_rotvec([0., 0., .3]).as_matrix()
    driver.quat[:] = Rotation.from_matrix(target).as_quat()
    initial = driver.q.copy()
    for _ in range(100): driver.step()
    assert np.max(np.abs(driver.q-initial)) > .03
    assert max(servo.measured_position_errors) < .008
    assert max(servo.measured_orientation_errors) < .3


def test_pose_jacobian_analytic_rotation_derivative_and_clutch_rotation_order(servo):
    driver = Driver(servo)
    driver.send(False)
    start_rotation = Rotation.from_euler('xyz', [.2,-.3,.1]).as_matrix()
    driver.quat[:] = Rotation.from_matrix(start_rotation).as_quat()
    driver.send(True)
    change = Rotation.from_rotvec([.1,.2,-.1]).as_matrix()
    driver.quat[:] = Rotation.from_matrix(change@start_rotation).as_quat()
    driver.step()
    for i in range(2):
        np.testing.assert_allclose(servo.targets[i,:3,:3], change@servo.tool_poses_start[i,:3,:3], atol=1e-12)
    for solver in servo.solvers:
        q = np.clip(np.array([-.3, .2, -.4, .1]), solver.model.lower, solver.model.upper)
        pose, jac = solver.kinematics(q)
        np.testing.assert_allclose(pose, solver.model.fk(q), atol=1e-12)
        for j in range(4):
            moved, _ = solver.kinematics(q+np.eye(4)[j]*1e-6)
            angular = Rotation.from_matrix(moved[:3,:3]@pose[:3,:3].T).as_rotvec()/1e-6
            np.testing.assert_allclose(angular, jac[3:,j], atol=1e-8)
