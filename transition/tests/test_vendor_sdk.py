"""Offline vendor-boundary tests: fake SDK only, no robot connections."""
from dataclasses import replace
import json
import hashlib
from pathlib import Path
import zipfile
from types import SimpleNamespace

import pytest

from physical_fixtures import config, manifest_data
from transition_autonomy.physical import vendor_sdk as vendor
from transition_autonomy.physical.manifest import Manifest


def complete_fixture_json(data):
    data['commissioning']['manifest_sha256']=None
    data['commissioning']['manifest_sha256']=Manifest.parse(json.dumps(data)).digest
    return json.dumps(data)


def setup_fixture():
    data = manifest_data(config())
    data['robot']['firmware'] = 'v1.6.1.1-release-01967-2026-04-27'
    data['sdk']['artifact_sha256'] = vendor.WHEEL_SHA256
    raw = complete_fixture_json(data)
    settings = vendor.ConnectionSettings('/fictional/test.whl', 'fake-nic', 99, '',
                                         'test-serial', data['robot']['firmware'])
    return settings, raw, Manifest.parse(raw)


def sdk_fixture(settings, calls):
    state = dict(mode=1, body=2, actions=(), serial=settings.expected_serial,
                 firmware=settings.expected_firmware, motion_result=None, stop_result=None,
                 motion_error=None, stop_error=None)
    class Factory:
        @staticmethod
        def Instance():
            return SimpleNamespace(Init=lambda *args: calls.append(('dds', *args)))
    class Client:
        def Init(self): calls.append(('Init',))
        def InitWithName(self, name): calls.append(('InitWithName', name))
        def GetRobotInfo(self):
            calls.append(('GetRobotInfo',))
            return SimpleNamespace(name='fake', nickname='', model='K1',
                version=state['firmware'], serial_number=state['serial'])
        def GetStatus(self):
            calls.append(('GetStatus',))
            return SimpleNamespace(current_mode=state['mode'], current_body_control=state['body'],
                                   current_actions=state['actions'])
        def GetFrameTransform(self, source, destination):
            calls.append(('GetFrameTransform', source, destination))
            return SimpleNamespace(position=SimpleNamespace(x=.1, y=.2, z=.3))
        def MoveHandEndEffectorV2(self, target, duration, hand):
            calls.append(('MoveHandEndEffectorV2', target.position, target.orientation, duration, hand))
            if state['motion_error']:
                raise state['motion_error']
            return state['motion_result']
        def StopHandEndEffector(self):
            calls.append(('StopHandEndEffector',))
            if state['stop_error']:
                raise state['stop_error']
            return state['stop_result']
    sdk = SimpleNamespace(ChannelFactory=Factory, B1LocoClient=Client,
        Frame=SimpleNamespace(kBody=0,kLeftHand=2,kRightHand=3),
        HandIndex=SimpleNamespace(kLeftHand=0,kRightHand=1), Posture=SimpleNamespace,
        Position=lambda *values: tuple(values), Orientation=lambda *values: tuple(values))
    return sdk, state


def test_query_port_has_no_motion_capabilities_and_preserves_unsynchronized_evidence():
    settings,_,_=setup_fixture(); calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.QueryPort(settings,loader=lambda _:sdk)
    assert not hasattr(port,'stop') and not hasattr(port,'move_profile')
    assert port.query_identity()['response']['serial_number']=='test-serial'
    assert port.query_status()['response']['current_body_control']==2
    hands=port.query_hand_positions()
    assert not hands['synchronized'] and not hands['source_progression_verified']
    assert all(not h['acquisition_timestamp_known'] for h in hands['hands'].values())
    assert [c[0] for c in calls]==['dds','Init','GetRobotInfo','GetStatus','GetFrameTransform','GetFrameTransform']


def test_stop_port_only_sends_stop_and_never_claims_measured_containment():
    settings,_,_=setup_fixture();calls=[];sdk,state=sdk_fixture(settings,calls)
    port=vendor.StopPort(settings,loader=lambda _:sdk)
    assert not hasattr(port,'query_status') and not hasattr(port,'move_profile')
    result=port.stop()
    assert result['rpc_returned'] and not result['measured_stop_confirmed']
    assert not result['pending_work_contained'] and not result['execution_complete']
    assert [c[0] for c in calls]==['dds','Init','StopHandEndEffector']
    state['stop_error']=RuntimeError('BadRequest: planner not active')
    with pytest.raises(RuntimeError,match='BadRequest'):
        port.request_stop()


@pytest.mark.parametrize('profile_id,hand',[('point_a',0),('point_b',0),('home',0)])
def test_exact_named_profile_and_final_admission_order(profile_id,hand):
    settings,raw,manifest=setup_fixture();calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk)
    assert not hasattr(port,'stop') and not hasattr(port,'query_status')
    def authorize():
        calls.append(('final_admission',))
        return True
    result=port.move_profile(profile_id,vendor.profile_digest(manifest,profile_id),before_dispatch=authorize)
    assert [c[0] for c in calls]==['dds','Init','GetRobotInfo','GetStatus','final_admission','MoveHandEndEffectorV2']
    assert calls[-1][1:]==((.1,.2,.3),(0.,0.,0.),1000,hand)
    assert result['manifest_digest']==manifest.digest and not result['execution_complete']
    assert port.enforcement_record is None and not result['pending_work_contained']


def test_right_hand_maps_explicitly_and_no_cpp_posture_constructor_assumed():
    settings,raw,_=setup_fixture();data=json.loads(raw)
    data['profiles']['point_a']['hand']='right'
    raw=complete_fixture_json(data);manifest=Manifest.parse(raw);calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk)
    port.move_profile('point_a',vendor.profile_digest(manifest,'point_a'),before_dispatch=lambda:True)
    assert calls[-1][-1]==1


@pytest.mark.parametrize('profile,digest',[('arbitrary','0'*64),('point_a','0'*64)])
def test_unknown_profile_or_tampered_digest_never_reaches_vendor_rpc(profile,digest):
    settings,raw,_=setup_fixture();calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk)
    with pytest.raises(ValueError):
        port.move_profile(profile,digest,before_dispatch=lambda:True)
    assert [c[0] for c in calls]==['dds','Init']


@pytest.mark.parametrize('field,value',[('serial','another'),('firmware','v1.6.1.2'),('mode',0),('body',6),('actions',(3,)),('mode',True)])
def test_changed_identity_mode_or_competing_action_denies_before_admission(field,value):
    settings,raw,manifest=setup_fixture();calls=[];sdk,state=sdk_fixture(settings,calls)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk);state[field]=value
    with pytest.raises((ValueError,RuntimeError)):
        port.move_profile('point_a',vendor.profile_digest(manifest,'point_a'),
                          before_dispatch=lambda:pytest.fail('must not admit'))
    assert all(c[0]!='MoveHandEndEffectorV2' for c in calls)


@pytest.mark.parametrize('return_value',[False,None,1])
def test_final_admission_denial_after_slow_reads_prevents_dispatch(return_value):
    settings,raw,manifest=setup_fixture();calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk)
    with pytest.raises(RuntimeError,match='denied'):
        port.move_profile('point_a',vendor.profile_digest(manifest,'point_a'),before_dispatch=lambda:return_value)
    assert [c[0] for c in calls][-1]=='GetStatus'


def test_expired_admission_exception_propagates_without_dispatch_or_retry():
    settings,raw,manifest=setup_fixture();calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk)
    def expire(): raise TimeoutError('owner expired during read')
    with pytest.raises(TimeoutError):
        port.move_profile('point_a',vendor.profile_digest(manifest,'point_a'),before_dispatch=expire)
    assert all(c[0]!='MoveHandEndEffectorV2' for c in calls)


@pytest.mark.parametrize('failure',[RuntimeError('timeout after submission'),None])
def test_unexpected_return_or_rpc_exception_is_unknown_and_not_retried(failure):
    settings,raw,manifest=setup_fixture();calls=[];sdk,state=sdk_fixture(settings,calls)
    state.update(motion_error=failure,motion_result=0)
    port=vendor.ProfileMotionPort(settings,raw,loader=lambda _:sdk)
    with pytest.raises(RuntimeError):
        port.move_profile('point_a',vendor.profile_digest(manifest,'point_a'),before_dispatch=lambda:True)
    assert [c[0] for c in calls].count('MoveHandEndEffectorV2')==1


@pytest.mark.parametrize('mutation',['disabled','artifact','binding','old_firmware','unresolved'])
def test_unreviewed_manifest_denies_before_sdk_load(mutation):
    settings,raw,_=setup_fixture();data=json.loads(raw)
    if mutation=='disabled':data['physical_actuation_enabled']=False
    if mutation=='artifact':data['sdk']['artifact_sha256']='0'*64
    if mutation=='binding':data['sdk']['network_binding']='wrong'
    if mutation=='old_firmware':
        data['robot']['firmware']='v1.3.1.0';settings=replace(settings,expected_firmware='v1.3.1.0')
    if mutation=='unresolved':data['controller']['support_record']=None
    with pytest.raises(ValueError):
        vendor.ProfileMotionPort(settings,complete_fixture_json(data),loader=lambda _:pytest.fail('must not load SDK'))


def test_port_close_is_local_and_prevents_reuse_without_sending_stop_or_mode_change():
    settings,_,_=setup_fixture();calls=[];sdk,_=sdk_fixture(settings,calls)
    port=vendor.StopPort(settings,loader=lambda _:sdk);port.close();port.close()
    with pytest.raises(RuntimeError,match='closed'):port.stop()
    assert [c[0] for c in calls]==['dds','Init']


@pytest.mark.parametrize('version,result',[('v1.3.1.1',True),('1.3.1',False),('v1.6.1.1-release-01967-2026-04-27',True),('unknown',False),('v1.2.9.9',False)])
def test_exact_documented_firmware_minimum(version,result):
    assert vendor.firmware_supports_endpoint(version) is result


def test_wrong_sdk_artifact_denies_before_import(tmp_path):
    wheel=tmp_path/vendor.WHEEL_NAME;wheel.write_bytes(b'unreviewed')
    with pytest.raises(ValueError,match='hash'):vendor.load_pinned_sdk(wheel)


@pytest.mark.parametrize('tamper',[None,'booster_robotics_sdk_python/__init__.py',
    'booster_robotics_sdk_python/_core.cpython-312-x86_64-linux-gnu.so',
    'booster_robotics_sdk_internal_python.cpython-312-x86_64-linux-gnu.so','shadow'])
def test_all_installed_package_files_verified_before_vendor_import(tmp_path,monkeypatch,tamper):
    payload={
        'booster_robotics_sdk_python/__init__.py':b'wrapper',
        'booster_robotics_sdk_python/_core.cpython-312-x86_64-linux-gnu.so':b'core',
        'booster_robotics_sdk_internal_python.cpython-312-x86_64-linux-gnu.so':b'internal',
        'booster_robotics_sdk_python-1.6.3.dist-info/RECORD':b'original record',
    }
    wheel=tmp_path/vendor.WHEEL_NAME;install=tmp_path/'installed'
    with zipfile.ZipFile(wheel,'w') as archive:
        for name,body in payload.items():
            archive.writestr(name,body)
            target=install/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(body)
    # Pip RECORD rewriting is expected; all executable payload bytes still bind.
    (install/'booster_robotics_sdk_python-1.6.3.dist-info/RECORD').write_bytes(b'installed record')
    if tamper and tamper!='shadow':(install/tamper).write_bytes(b'changed')
    monkeypatch.setattr(vendor,'WHEEL_SHA256',hashlib.sha256(wheel.read_bytes()).hexdigest())
    monkeypatch.setattr(vendor.sys,'version_info',(3,12))
    monkeypatch.setattr(vendor.platform,'system',lambda:'Linux')
    monkeypatch.setattr(vendor.platform,'machine',lambda:'x86_64')
    monkeypatch.setattr(vendor.importlib.metadata,'distribution',lambda _:SimpleNamespace(
        version='1.6.3',locate_file=lambda name:install/name))
    def find(module):
        name=(module+'/__init__.py' if module==vendor.DISTRIBUTION
              else module+'.cpython-312-x86_64-linux-gnu.so')
        return SimpleNamespace(origin=str((tmp_path/'shadow' if tamper=='shadow' else install)/name))
    monkeypatch.setattr(vendor.importlib.util,'find_spec',find)
    imported=[]
    monkeypatch.setattr(vendor.importlib,'import_module',lambda name:imported.append(name) or 'verified-module')
    if tamper:
        with pytest.raises(ValueError):vendor.load_pinned_sdk(wheel)
        assert not imported
    else:
        assert vendor.load_pinned_sdk(wheel)=='verified-module'
        assert imported==[vendor.DISTRIBUTION]
