"""Explicitly fictional trusted evidence for offline protocol tests only."""
from dataclasses import asdict, replace
import json

from transition_autonomy.backends.booster_backend import K1Config, PointProfile
from transition_autonomy.physical.manifest import Manifest
from transition_autonomy.physical.admission import Approval, ApprovalRegistry, ReleaseGate, SiteEvidence, TaskGrant
from transition_autonomy.physical.sampling import Acquisition, ObservationBundle


def config(enabled=True):
    profiles = {name: PointProfile('left', (.1, .2, .3), (0., 0., 0.), 1000, label, .01, .1)
                for name, label in [('point_a', 'A'), ('point_b', 'B'), ('home', None)]}
    return K1Config('test-serial', 'test-fw', 1, 2, profiles, 'test-receipt', physical_actuation_enabled=enabled)


def manifest_data(c):
    return dict(schema_version=1, deployment_id='offline-fixture',
        robot=dict(serial=c.expected_serial, firmware=c.expected_firmware, model='K1'),
        control_host=dict(host_id='fake-host', os_release='fake-os', python_abi='fake-abi', cpu_architecture='fake-arch'),
        sdk=dict(distribution='booster_robotics_sdk_python', version='1.6.3', artifact_sha256='0'*64,
                 network_binding='fake-nic', vendor_dds_domain=99, robot_name=''),
        physical_actuation_enabled=c.physical_actuation_enabled,
        controller=dict(allowed_mode=c.allowed_mode, allowed_body_control=c.allowed_body_control,
                        support_record='fake-support', independent_stop_record='fake-stop'),
        ownership=dict(enforcement_record='fake-enforcement', watchdog_record='fake-watchdog'),
        observation=dict(max_age_s=c.max_state_age, max_gap_s=.25, max_skew_s=.02,
                         max_transport_uncertainty_s=.01, max_arm_speed_rad_s=c.max_arm_speed, calibration_digest='1'*64),
        profiles={name: asdict(profile) for name, profile in c.profiles.items()},
        commissioning=dict(receipt_id=c.commissioning_receipt, manifest_sha256=None,
                           reviewer='fake-reviewer', approved_at=100., expires_at=200.))


def make_manifest(c):
    data = manifest_data(c)
    manifest = Manifest.parse(json.dumps(data))
    data['commissioning']['manifest_sha256'] = manifest.digest
    return Manifest.parse(json.dumps(data))


def bundle(at=1., sequence=1, **changes):
    components = tuple(Acquisition(name, at, at, at, at, 'fake acquisition; no hardware evidence')
                       for name in ['joints', 'identity', 'status', 'left_hand', 'right_hand'])
    return replace(ObservationBundle('sample-'+str(sequence), 'fake-boot', 'fake-owner', sequence,
                   at, components, .001, True, '1'*64, 'fake-native', 20.,
                   position_uncertainty_m=.0001, velocity_uncertainty_rad_s=.0001), **changes)


def make_gate(c, transport):
    m = make_manifest(c)
    registry = ApprovalRegistry([Approval('test-receipt', m.digest, 'fake-reviewer',
        'operational_release', 100., 200., frozenset(c.profiles), 'fake-enforcement',
        starting_regions=frozenset({'fake-region'}), minimum_extra_budget_s=.1,
        approved_transitions=frozenset((name, 'fake-region') for name in c.profiles))], trusted_issuers={'fake-reviewer'})
    def site():
        b = transport.sample.bundle
        return SiteEvidence('fake-monitor', b.owner_epoch, transport.sample.sampled_at, 20., 'fake-enforcement',
             'fake-watchdog', 'fake-support', 'fake-stop', '1'*64, 'fake-region', b.bundle_id, b.boot_id)
    return ReleaseGate(m, registry, site_resolver=site,
        grant_resolver=lambda _: TaskGrant('authority', 1, 20., frozenset({'point-a'})),
        clock=lambda: transport.sample.sampled_at, wall_clock=lambda: 150.,
        trusted_monitor_issuers={'fake-monitor'}, extra_budget_s=.1)


def finish(backend):
    """Measured fake stop and explicit fake containment; never used outside tests."""
    if backend._active is None:
        backend.close()
        return
    t = backend.transport
    t.fail_stop = False
    t.sample = replace(t.sample, sampled_at=max(1., t.sample.sampled_at), arm_velocities=(0.,)*8,
                       serial=backend.config.expected_serial)
    backend._last_bundle = None
    backend.request_stop(t.sample.sampled_at)
    backend.poll(t.sample.sampled_at)
    t.sample = replace(t.sample, sampled_at=t.sample.sampled_at+.2)
    backend.poll(t.sample.sampled_at)
    result = backend.shutdown(t.sample.sampled_at)
    assert result['closed']
