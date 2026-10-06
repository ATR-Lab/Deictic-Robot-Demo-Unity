import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import pytest

DEPLOYMENT=Path(__file__).resolve().parents[1]/'deployment'
sys.path.insert(0,str(DEPLOYMENT))
spec=importlib.util.spec_from_file_location('k1_ros_observer',DEPLOYMENT/'k1_ros_observer.py')
observer=importlib.util.module_from_spec(spec);spec.loader.exec_module(observer)


def test_read_only_query_allowlist_cannot_select_motion_or_raw_api():
    assert observer.read_request('robot_status')==(2018,'')
    assert observer.read_request('body_to_right_hand')==(2011,'{"src":0,"dst":3}')
    assert {observer.read_request(name)[0] for name in observer.READ_QUERIES}=={2011,2018,2022}
    for bad in ('move','stop','ChangeMode','2009','UpperBodyCustomControl'):
        with pytest.raises(ValueError):observer.read_request(bad)


def test_publication_stamp_supports_jazzy_metadata_without_inventing_missing_time():
    assert observer.publication_stamp({'source_timestamp':1234})==1234
    assert observer.publication_stamp(SimpleNamespace(source_timestamp=5678))==5678
    for missing in ({},{'source_timestamp':0},{'source_timestamp':True},object()):
        assert observer.publication_stamp(missing) is None


@pytest.mark.parametrize('raw',['[]','{"x":1,"x":2}','{"x":NaN}','{"x":1e999}','x'*65537])
def test_rpc_json_rejects_ambiguous_or_nonfinite_payloads(raw):
    with pytest.raises(ValueError):observer.strict_object(raw)


def motor(**changes):
    return SimpleNamespace(**dict({'q':0.,'dq':0.,'ddq':0.,'tau_est':0.,'mode':1,'temperature':30,'lost':0,'reserve':[0,500]},**changes))


def test_reported_motor_faults_are_preserved_and_not_release_approval():
    motors=[motor() for _ in range(22)]
    motors[6]=motor(lost=4,reserve=[3,480])
    result=observer.motor_payload(SimpleNamespace(motor_state_serial=motors))
    assert result['arm_motor_health_clear'] is False
    assert result['motors_serial'][6]['error_code']==3
    assert result['motors_serial'][6]['lost']==4
    assert 'commissioning' in result['health_interpretation']
    motors[0]=motor(q=float('nan'))
    with pytest.raises(ValueError):observer.motor_payload(SimpleNamespace(motor_state_serial=motors))


def test_snapshot_replacement_is_complete_and_invalid_payload_keeps_previous(tmp_path):
    path=tmp_path/'latest.json'
    observer.atomic_snapshot(path,{'sequence':1,'motion_capability':False})
    with pytest.raises(ValueError):observer.atomic_snapshot(path,{'sequence':float('nan')})
    assert json.loads(path.read_text())['sequence']==1
    observer.atomic_snapshot(path,{'sequence':2,'motion_capability':False})
    assert json.loads(path.read_text())['sequence']==2
    assert list(tmp_path.iterdir())==[path]
