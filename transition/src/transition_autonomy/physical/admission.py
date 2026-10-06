"""Admission contracts for a separately commissioned command-enforcing gateway.

Resolvers are injected by the trusted local composition root, never accepted as
UI/ROS payloads. This module does not provide device fencing or invent a permit.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from types import MappingProxyType

from .manifest import Manifest, finite, text


@dataclass(frozen=True)
class Approval:
    receipt_id: str
    manifest_digest: str
    issuer: str
    permit_class: str
    approved_at: float
    expires_at: float
    profiles: frozenset[str]
    enforcement_record: str
    revoked: bool = False
    starting_regions: frozenset[str] = frozenset()
    minimum_extra_budget_s: float = 0.
    commissioning_command_ids: frozenset[str] = frozenset()
    approved_transitions: frozenset[tuple[str, str]] = frozenset()

    def __post_init__(self):
        for name in ('profiles', 'starting_regions', 'commissioning_command_ids', 'approved_transitions'):
            object.__setattr__(self, name, frozenset(getattr(self, name)))


class ApprovalRegistry:
    """Detached trusted-local records, with revocation resolved on each check.

    Only trusted server code may construct/replace this registry. A deployment
    must verify the issuer through its local approval service before construction.
    A manifest's self-declared reviewer or matching hash never creates approval.
    """
    def __init__(self, approvals=(), *, trusted_issuers=()):
        self._issuers = frozenset(trusted_issuers)
        records = {}
        for approval in approvals:
            if not isinstance(approval, Approval) or approval.receipt_id in records:
                raise ValueError('Approval record invalid or duplicated')
            records[approval.receipt_id] = approval
        self._records = MappingProxyType(records)
        self._revoked = set()

    def revoke(self, receipt_id):
        self._revoked.add(receipt_id)

    def require(self, manifest, profile, wall_now, *, command_id=None):
        manifest.require_complete()
        receipt = manifest.payload['commissioning']['receipt_id']
        record = self._records.get(receipt)
        if record is None or record.issuer not in self._issuers:
            raise RuntimeError('No trusted commissioning approval')
        finite(wall_now, 'wall clock')
        if (type(record.revoked) is not bool or record.revoked or receipt in self._revoked
                or record.manifest_digest != manifest.digest
                or record.permit_class not in ('bounded_commissioning', 'operational_release')
                or not finite(record.approved_at, 'approval time') <= wall_now < finite(record.expires_at, 'approval expiry')
                or not record.profiles or not record.profiles <= {'point_a', 'point_b', 'home'}
                or not record.starting_regions
                or (profile is not None and profile not in record.profiles)
                or record.enforcement_record != manifest.payload['ownership']['enforcement_record']):
            raise RuntimeError('Approval expired, revoked, mismatched or out of scope')
        finite(record.minimum_extra_budget_s, 'reviewed operation margin', positive=True)
        if record.permit_class == 'bounded_commissioning' and (not record.commissioning_command_ids
                or (command_id is not None and command_id not in record.commissioning_command_ids)):
            raise RuntimeError('Commissioning permit requires exact bounded command IDs')
        if not record.approved_transitions or any(profile not in record.profiles or region not in record.starting_regions
                                                 for profile, region in record.approved_transitions):
            raise RuntimeError('Profile transitions have not been approved')
        return record


@dataclass(frozen=True)
class SiteEvidence:
    issuer: str
    owner_epoch: str
    observed_at: float
    expires_at: float
    enforcement_record: str
    watchdog_record: str
    support_record: str
    independent_stop_record: str
    calibration_digest: str
    starting_region_id: str
    observation_bundle_id: str
    boot_id: str
    failed_conditions: tuple[str, ...] = ()

    def __post_init__(self):
        object.__setattr__(self, 'failed_conditions', tuple(self.failed_conditions))


@dataclass(frozen=True)
class TaskGrant:
    authority_id: str
    revision: int
    expires_at: float
    skills: frozenset[str]
    revoked: bool = False

    def __post_init__(self):
        object.__setattr__(self, 'skills', frozenset(self.skills))


@dataclass(frozen=True)
class PhysicalAdmission:
    command_json: str
    manifest_digest: str
    owner_epoch: str
    owner_expires_at: float
    authority_id: str
    authority_revision: int
    authority_expires_at: float
    remaining_budget_s: float
    starting_region_id: str
    observation_bundle_id: str
    boot_id: str
    accepted_at: float
    enforcement_record: str


class ReleaseGate:
    def __init__(self, manifest: Manifest, registry: ApprovalRegistry, *,
                 site_resolver, grant_resolver, clock, wall_clock,
                 trusted_monitor_issuers, extra_budget_s):
        self.manifest, self.registry = manifest, registry
        self.site_resolver, self.grant_resolver = site_resolver, grant_resolver
        self.clock, self.wall_clock = clock, wall_clock
        self.trusted_monitor_issuers = frozenset(trusted_monitor_issuers)
        self.extra_budget_s = finite(extra_budget_s, 'dispatch/terminal margin', positive=True)

    def _site(self, sample, now):
        site = self.site_resolver()
        if not isinstance(site, SiteEvidence) or site.issuer not in self.trusted_monitor_issuers:
            raise RuntimeError('No trusted expiring site monitor')
        manifest = self.manifest.payload
        if (site.failed_conditions or not finite(site.observed_at, 'site time') <= now < finite(site.expires_at, 'owner expiry')
                or now-site.observed_at > manifest['observation']['max_age_s']):
            raise RuntimeError('Site/owner evidence expired or failed')
        matches = ((site.enforcement_record, manifest['ownership']['enforcement_record']),
                   (site.watchdog_record, manifest['ownership']['watchdog_record']),
                   (site.support_record, manifest['controller']['support_record']),
                   (site.independent_stop_record, manifest['controller']['independent_stop_record']),
                   (site.calibration_digest, manifest['observation']['calibration_digest']))
        if any(actual != expected for actual, expected in matches):
            raise RuntimeError('Site evidence does not match approved deployment')
        for field in ('owner_epoch', 'starting_region_id', 'observation_bundle_id', 'boot_id'):
            text(getattr(site, field), field)
        # Sample evidence is produced by the bounded sampler and bound to the
        # same commissioned monitor/owner. A legacy site_ready Boolean cannot pass.
        bundle = getattr(sample, 'bundle', None)
        if bundle is None:
            raise RuntimeError('Acquisition/registration evidence bundle missing')
        bundle.validate(now, manifest['observation'])
        if (bundle.bundle_id != site.observation_bundle_id or bundle.owner_epoch != site.owner_epoch
                or bundle.boot_id != site.boot_id or bundle.calibration_digest != site.calibration_digest):
            raise RuntimeError('Observation, registration and owner epochs disagree')
        return site

    def arm(self, sample, *, enforcement_record):
        approval = self.registry.require(self.manifest, None, self.wall_clock())
        now = self.clock()
        site = self._site(sample, now)
        if enforcement_record != site.enforcement_record:
            raise RuntimeError('Transport has no matching commissioned expiry/owner enforcement')
        if site.starting_region_id not in approval.starting_regions or self.extra_budget_s < approval.minimum_extra_budget_s:
            raise RuntimeError('Starting region or operation budget is unapproved')
        return site.owner_epoch

    def admit(self, command, profile, sample, *, enforcement_record, expected_owner):
        profile_id = command.parameters.get('profile')
        approval = self.registry.require(self.manifest, profile_id, self.wall_clock(), command_id=command.command_id)
        now = self.clock()
        site = self._site(sample, now)
        if site.owner_epoch != expected_owner or enforcement_record != site.enforcement_record:
            raise RuntimeError('Owner changed or transport enforcement unavailable')
        if (profile_id, site.starting_region_id) not in approval.approved_transitions or self.extra_budget_s < approval.minimum_extra_budget_s:
            raise RuntimeError('Starting region or operation budget is unapproved')
        grant = self.grant_resolver(command.authority_id)
        if (not isinstance(grant, TaskGrant) or grant.revoked is not False
                or type(grant.revision) is not int or grant.revision != command.authority_revision
                or grant.authority_id != command.authority_id or command.skill_id not in grant.skills):
            raise RuntimeError('Task authority revoked, mismatched or out of scope')
        budget = profile.duration_ms/1000 + profile.settling_seconds + self.extra_budget_s
        if (not now + budget <= finite(command.deadline, 'command deadline') <= finite(grant.expires_at, 'authority expiry')
                or finite(command.issued_at, 'command issue') > now or command.deadline > site.expires_at):
            raise RuntimeError('Insufficient live deadline/authority/owner budget')
        return PhysicalAdmission(json.dumps(command.to_dict(), sort_keys=True, separators=(',', ':'), allow_nan=False),
            self.manifest.digest, site.owner_epoch, site.expires_at, grant.authority_id,
            grant.revision, grant.expires_at, budget, site.starting_region_id,
            site.observation_bundle_id, site.boot_id, now, site.enforcement_record)

    def check_running(self, command, sample, *, expected_owner):
        self.registry.require(self.manifest, command.parameters['profile'], self.wall_clock(), command_id=command.command_id)
        now = self.clock()
        site = self._site(sample, now)
        grant = self.grant_resolver(command.authority_id)
        if (site.owner_epoch != expected_owner or not isinstance(grant, TaskGrant)
                or grant.revoked is not False or grant.authority_id != command.authority_id
                or grant.revision != command.authority_revision or command.skill_id not in grant.skills
                or not now < min(grant.expires_at, command.deadline)):
            raise RuntimeError('Running command lost owner/task authority')
