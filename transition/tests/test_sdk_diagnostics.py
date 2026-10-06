"""Tests exercise only fake SDK objects and disposable workers, never a robot."""
import importlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import zipfile

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'deployment'))
diagnostic = importlib.import_module('k1_sdk_diagnostics')


def sdk_fixture(calls):
    class Channel:
        def Init(self, domain, binding): calls.append(('channel', domain, binding))
    class Factory:
        @staticmethod
        def Instance(): return Channel()
    class Client:
        def __init__(self): calls.append(('client',))
        def Init(self): calls.append(('default_init',))
        def InitWithName(self, name): calls.append(('named_init', name))
        def GetRobotInfo(self):
            calls.append(('GetRobotInfo',))
            return SimpleNamespace(name='fake', nickname='', version='fake-fw', model='K1',
                serial_number='fake-serial')
        def GetStatus(self):
            calls.append(('GetStatus',))
            return SimpleNamespace(current_mode=1, current_body_control=2, current_actions=[3])
    return SimpleNamespace(ChannelFactory=Factory, B1LocoClient=Client)


@pytest.mark.parametrize('query', diagnostic.READ_ONLY_QUERIES)
def test_only_explicit_query_and_connection_calls_are_possible(query):
    calls=[]
    result=diagnostic.query_using_sdk(sdk_fixture(calls), query,
        binding='fake-nic',domain_id=99,robot_name='',discovery_wait_s=0)
    assert calls == [('channel',99,'fake-nic'),('client',),('default_init',),(query,)]
    assert result['ok'] and not result['acquisition_timestamp_known']
    if query == 'GetRobotInfo':
        assert set(result['response']) == {'name','nickname','version','model','serial_number'}


def test_arbitrary_query_rejected_before_any_client_or_channel():
    calls=[]
    with pytest.raises(ValueError, match='Only'):
        diagnostic.query_using_sdk(sdk_fixture(calls),'unapproved_operation',
            binding='fake',domain_id=99,robot_name='',discovery_wait_s=0)
    assert not calls


def test_old_sdk_missing_read_api_reports_failure_without_fallback():
    calls=[]
    sdk=sdk_fixture(calls)
    del sdk.B1LocoClient.GetRobotInfo
    with pytest.raises(AttributeError):
        diagnostic.query_using_sdk(sdk,'GetRobotInfo',binding='fake',domain_id=99,
            robot_name='fake-name',discovery_wait_s=0)
    assert calls[-1] == ('named_init','fake-name')


def stuck_worker(connection, query, settings):
    time.sleep(30)


def test_stuck_sdk_query_has_bounded_process_lifetime():
    started=time.monotonic()
    result=diagnostic.bounded_query('GetRobotInfo',{},.1,worker=stuck_worker)
    assert not result['ok'] and result['error_type']=='TimeoutError'
    assert result['worker_exitcode'] is not None
    assert time.monotonic()-started < 1.5


def test_wrong_wheel_refused_before_any_sdk_import(tmp_path):
    wheel=tmp_path/diagnostic.WHEEL_NAME
    wheel.write_bytes(b'unreviewed')
    with pytest.raises(ValueError, match='hash'):
        diagnostic.verify_install(wheel)


@pytest.fixture
def package_layout(tmp_path):
    # Mirrors the downloaded pinned wheel's real package-wrapper layout,
    # including its separate top-level internal extension and example files.
    payload = {
        'booster_robotics_sdk_python/__init__.py': b'from ._core import *\nfrom .arm_controller import ArmController\n',
        'booster_robotics_sdk_python/_core.cpython-312-x86_64-linux-gnu.so': b'core fixture bytes',
        'booster_robotics_sdk_python/arm_controller.py': b'class ArmController: pass\n',
        'booster_robotics_sdk_python/move_controller.py': b'class MoveController: pass\n',
        'booster_robotics_sdk_internal_python.cpython-312-x86_64-linux-gnu.so': b'internal fixture bytes',
        'python_example/sdk_pybind_b1_example.py': b'example fixture; never execute',
        'booster_robotics_sdk_python-1.6.3.dist-info/METADATA': b'Name: booster_robotics_sdk_python\nVersion: 1.6.3\n',
        'booster_robotics_sdk_python-1.6.3.dist-info/RECORD': b'wheel record',
    }
    wheel=tmp_path/'package.whl'; installed=tmp_path/'site-packages'
    with zipfile.ZipFile(wheel,'w') as archive:
        for name,data in payload.items():
            archive.writestr(name,data)
            path=installed/name; path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    (installed/'booster_robotics_sdk_python-1.6.3.dist-info/RECORD').write_bytes(b'pip rewrites RECORD')
    distribution=SimpleNamespace(locate_file=lambda name: installed/name)
    def find_spec(name):
        origin=(installed/'booster_robotics_sdk_python/__init__.py' if name==diagnostic.DISTRIBUTION
                else installed/(name+'.cpython-312-x86_64-linux-gnu.so'))
        return SimpleNamespace(origin=str(origin))
    return wheel, installed, distribution, find_spec, payload


def test_package_wrapper_and_all_wheel_payloads_are_checked_without_import(package_layout):
    wheel,installed,distribution,finder,payload=package_layout
    result=diagnostic.verify_wheel_files(wheel,distribution,find_spec=finder)
    assert result['verified_file_count']==len(payload)-1
    assert len(result['native_extensions'])==2
    assert str(installed/'booster_robotics_sdk_python')==result['package_path']


@pytest.mark.parametrize('member', [
    'booster_robotics_sdk_python/__init__.py',
    'booster_robotics_sdk_python/arm_controller.py',
    'booster_robotics_sdk_python/_core.cpython-312-x86_64-linux-gnu.so',
    'booster_robotics_sdk_internal_python.cpython-312-x86_64-linux-gnu.so',
    'python_example/sdk_pybind_b1_example.py',
])
def test_modified_wrapper_native_or_other_payload_is_rejected(package_layout, member):
    wheel,installed,distribution,finder,_=package_layout
    (installed/member).write_bytes(b'modified')
    with pytest.raises(ValueError,match='differs'):
        diagnostic.verify_wheel_files(wheel,distribution,find_spec=finder)


def test_shadowed_package_is_rejected_without_import(package_layout):
    wheel,installed,distribution,finder,_=package_layout
    with pytest.raises(ValueError,match='shadowed'):
        diagnostic.verify_wheel_files(wheel,distribution,find_spec=lambda name:SimpleNamespace(origin='/other/'+name))


def test_preflight_failure_records_no_physical_readiness(tmp_path, monkeypatch):
    # An isolated installation tests the missing-wheel path independently of the
    # real incident hold. No native worker may run, even if preflight regresses.
    monkeypatch.setattr(diagnostic, '__file__', str(tmp_path/'deployment/k1_sdk_diagnostics.py'))
    monkeypatch.setattr(diagnostic, 'bounded_query', lambda *args, **kwargs: pytest.fail('Unexpected SDK worker'))
    output=tmp_path/'diagnostic.json'
    monkeypatch.setattr(sys,'argv',['diagnostic','--wheel',str(tmp_path/'missing.whl'),
        '--network-binding','fake','--domain-id','99','--robot-name','','--output',str(output)])
    assert diagnostic.main()==1
    result=json.loads(output.read_text())
    assert not result['physical_ready'] and not result['motion_capability']
    assert not result['read_only_rpc_connected'] and result['results']==[]


def test_incident_hold_refuses_before_argument_parsing_or_sdk_preflight(tmp_path, monkeypatch):
    marker = tmp_path/'.runtime/k1-diagnostics-hold.json'
    marker.parent.mkdir()
    marker.write_text('{"status":"held"}')
    monkeypatch.setattr(diagnostic, '__file__', str(tmp_path/'deployment/k1_sdk_diagnostics.py'))
    monkeypatch.setattr(sys, 'argv', ['diagnostic'])
    monkeypatch.setattr(diagnostic, 'verify_install', lambda *args: pytest.fail('Unexpected SDK preflight'))
    monkeypatch.setattr(diagnostic, 'bounded_query', lambda *args, **kwargs: pytest.fail('Unexpected SDK worker'))
    with pytest.raises(SystemExit, match='diagnostics held'):
        diagnostic.main()
