import json
from pathlib import Path
import numpy as np
import pytest
from deictic_control.se3 import from_pose, inverse, to_pose, rotation
from deictic_control.fusion import Observation, FusionState, FusionConfig, SimulationReference, GtsamPoseGraph
from deictic_control.kinematics import ArmModel, make_plan, segment_intersects_box


ROOT = Path(__file__).resolve().parents[4]


def observation(stamp=100., world=None, headset=None, camera=None, source='simulation_ground_truth'):
    world = from_pose([.4, -.2, .3, 0, 0, 0, 1]) if world is None else world
    headset = from_pose([.1, .2, .3, 0, 0, 0, 1]) if headset is None else headset
    camera = from_pose([-.2, .1, .4, 0, 0, 0, 1]) if camera is None else camera
    return Observation(stamp, headset, camera, inverse(camera)@world@headset, 90, 100, .1, source)


def test_se3_roundtrip_and_direction():
    for angle in [0., .3, np.pi-.000001, np.pi]:
        t = np.eye(4)
        t[:3,:3] = rotation([1,2,3], angle)
        t[:3,3] = [.2,-.3,.1]
        np.testing.assert_allclose(from_pose(to_pose(t)), t, atol=1e-9)
        np.testing.assert_allclose(inverse(t)@t, np.eye(4), atol=1e-9)
    obs = observation()
    np.testing.assert_allclose(obs.camera@obs.relative@inverse(obs.headset), from_pose([.4,-.2,.3,0,0,0,1]))


def test_registration_parser_rejects_malformed_and_nonfinite():
    data = dict(schema_version=1, stamp=100., headset_pose=[0,0,0,0,0,0,1],
                camera_pose=[0,0,0,0,0,0,1], camera_from_headset=[0,0,0,0,0,0,1],
                inliers=20, matches=25, median_reprojection_error=.4, source='superpoint_pnp')
    assert Observation.parse(json.dumps(data)).inliers == 20
    for payload in ['null', '[]', '42']:
        with pytest.raises(ValueError, match='JSON object'):
            Observation.parse(payload)
    for field, value in [('inliers', 26), ('matches', True), ('stamp', float('nan')),
                         ('camera_pose', [0,0,0,0,0,0,0]), ('median_reprojection_error', -1)]:
        with pytest.raises(ValueError):
            Observation.parse(dict(data, **{field:value}))


def test_confidence_failure_freezes_transform_and_blocks_commit():
    state = FusionState(SimulationReference(), FusionConfig(allow_synthetic=True))
    assert state.ingest(observation(), 100.01)
    accepted = state.transform.copy()
    bad = observation(100.1); bad.inliers = 3
    assert not state.ingest(bad, 100.11)
    assert not state.snapshot(100.2)['can_commit']
    np.testing.assert_array_equal(state.transform, accepted)
    assert state.ingest(observation(100.3), 100.31)
    assert state.version == 1  # Identical alignment does not invalidate preview.
    assert not state.snapshot(101.31)['can_commit']
    assert state.reason == 'registration_stale'


def test_timestamp_jump_and_source_guards():
    state = FusionState(SimulationReference(), FusionConfig(allow_synthetic=True))
    assert not state.ingest(observation(102), 100)
    assert state.ingest(observation(100), 100.01)
    assert not state.ingest(observation(99.99), 100.01)
    jumped = observation(100.2, world=from_pose([1.,-.2,.3,0,0,0,1]))
    assert not state.ingest(jumped, 100.21)
    assert state.reason == 'registration_jump_rejected'
    assert not FusionState().ingest(observation(), 100.)
    real_mode = FusionState(SimulationReference())
    assert not real_mode.ingest(observation(), 100.)


def test_headset_epoch_reset_freezes_old_pose_and_rejects_old_captures():
    state = FusionState(SimulationReference(), FusionConfig(allow_synthetic=True))
    assert state.ingest(observation(100), 100.01)
    old_transform, old_version = state.transform.copy(), state.version
    state.reset('headset_recentered', minimum_stamp=100.2)
    assert state.version > old_version and state.epoch == 1
    assert not state.snapshot(100.21)['can_commit']
    np.testing.assert_array_equal(state.transform, old_transform)
    assert not state.ingest(observation(100.1), 100.22)
    assert state.reason == 'pre_reset_registration'
    new_world = from_pose([1.2, -.4, .5, 0, 0, 0, 1])
    assert state.ingest(observation(100.3, world=new_world), 100.31)
    np.testing.assert_allclose(state.transform, new_world)
    assert state.snapshot(100.32)['can_commit']


def test_gtsam_graph_recovers_noncommuting_frame_chain():
    pytest.importorskip('gtsam')
    backend = GtsamPoseGraph()
    truth = np.eye(4); truth[:3,:3] = rotation([1,2,3], .35); truth[:3,3] = [.4,-.2,.3]
    for i in range(4):
        headset, camera = np.eye(4), np.eye(4)
        headset[:3,:3] = rotation([1,0,1], i*.08); headset[:3,3] = [i*.03,.2,.3]
        camera[:3,:3] = rotation([0,1,1], -i*.09); camera[:3,3] = [-.2,.1+i*.02,.4]
        result = backend.update(observation(100+i*.1, truth, headset, camera))
        np.testing.assert_allclose(result, truth, atol=1e-6)
    assert backend.index == 4


def test_gtsam_corrects_vio_drift_in_output_transform():
    pytest.importorskip('gtsam')
    backend = GtsamPoseGraph()
    truth = np.eye(4); truth[:3,3] = [.4,-.2,.3]
    for i in range(8):
        h_true, h_vio, camera = np.eye(4), np.eye(4), np.eye(4)
        h_true[:3,3] = [.03*i,.1,.2]
        h_vio[:3,3] = h_true[:3,3] + [.004*i,0,0]
        camera[:3,3] = [.2,-.3,.1]
        relative = inverse(camera)@truth@h_true
        obs = Observation(100+i*.1, h_vio, camera, relative, 90,100,.1,'superpoint_pnp')
        result = backend.update(obs)
    raw_error = np.linalg.norm((truth@h_vio)[:3,3]-(truth@h_true)[:3,3])
    fused_error = np.linalg.norm((result@h_vio)[:3,3]-(truth@h_true)[:3,3])
    assert fused_error < raw_error*.5


def test_gtsam_reset_removes_previous_origin_constraints():
    pytest.importorskip('gtsam')
    backend = GtsamPoseGraph(max_keyframes=10)
    backend.update(observation())
    backend.reset()
    assert backend.index == 0 and backend.previous is None and not backend.callbacks
    assert backend.max_keyframes == 10
    new_world = from_pose([2., 1., -.5, 0, 0, 0, 1])
    np.testing.assert_allclose(backend.update(observation(101, world=new_world)), new_world, atol=1e-6)


@pytest.fixture
def model():
    return ArmModel(ROOT/'models/K1/K1_22dof.urdf')


def test_k1_fk_and_reachable_plan_limits_timing(model):
    assert model.names[0] == 'aaright_shoulder_pitch_joint'  # Actual vendor spelling.
    np.testing.assert_allclose(model.fk([0,0,0,0])[:3,3], [.0025,-.410928,.171], atol=1e-9)
    current = np.array([0.,1.,0.,0.])
    target = model.fk([-.2,.8,.3,.45])[:3,3]
    plan = make_plan(model, current, target)
    assert np.linalg.norm(model.fk(plan.positions[-1])[:3,3]-target) <= .002
    assert np.all(np.diff(plan.times)>0)
    assert np.max(np.abs(plan.velocities)) <= .35+1e-8
    assert np.max(np.abs(plan.accelerations)) <= .7+1e-8
    assert np.all(plan.positions >= model.lower) and np.all(plan.positions <= model.upper)
    np.testing.assert_allclose(plan.positions[0], current)
    np.testing.assert_allclose(plan.velocities[[0,-1]], 0, atol=1e-10)


def test_unreachable_and_collision_goals_rejected(model):
    current = [0.,1.,0.,0.]
    with pytest.raises(ValueError, match='Unreachable'):
        model.inverse_position([2,2,2], current)
    target = model.fk([-.2,.8,.3,.45])[:3,3]
    with pytest.raises(ValueError, match='table'):
        make_plan(model, current, target, table_top=.1)
    with pytest.raises(ValueError, match='obstacle'):
        make_plan(model, current, target, obstacles=[[-1,-1,-1,1,1,1]])
    assert segment_intersects_box(np.array([-1.,0,0]), np.array([1.,0,0]), [-.1,-.1,-.1,.1,.1,.1])
    assert not segment_intersects_box(np.array([-1.,1,0]), np.array([1.,1,0]), [-.1,-.1,-.1,.1,.1,.1])


def test_explicit_joint_return_preserves_exact_goal_and_collision_guards(model):
    from deictic_control.kinematics import make_joint_plan
    current, goal = [-.2, .8, .3, .45], [0., .5, 0., 0.]
    plan = make_joint_plan(model, current, goal)
    np.testing.assert_allclose(plan.positions[-1], goal, atol=1e-10)
    assert np.max(np.abs(plan.velocities)) <= .35+1e-8
    assert np.max(np.abs(plan.accelerations)) <= .7+1e-8
    with pytest.raises(ValueError, match='Goal joints outside'):
        make_joint_plan(model, current, model.upper+.1)
    with pytest.raises(ValueError, match='Invalid current/goal'):
        make_joint_plan(model, current, [np.nan, .5, 0, 0])
    with pytest.raises(ValueError, match='obstacle'):
        make_joint_plan(model, current, goal, obstacles=[[-1,-1,-1,1,1,1]])
