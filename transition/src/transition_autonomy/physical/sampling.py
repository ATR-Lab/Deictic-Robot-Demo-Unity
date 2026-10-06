"""Immutable acquisition evidence and bounded cache; no SDK or motion methods."""
from __future__ import annotations

from dataclasses import dataclass
import threading

from .manifest import finite, text


@dataclass(frozen=True)
class Acquisition:
    name: str
    earliest: float
    latest: float
    request_start: float
    response_end: float
    semantics: str

    def validate(self):
        text(self.name, 'component')
        text(self.semantics, 'acquisition semantics')
        for name in ('earliest', 'latest', 'request_start', 'response_end'):
            finite(getattr(self, name), name)
        if not self.earliest <= self.latest <= self.response_end or self.request_start > self.response_end:
            raise ValueError('Invalid acquisition interval')


@dataclass(frozen=True)
class ObservationBundle:
    bundle_id: str
    boot_id: str
    owner_epoch: str
    sequence: int
    published_at: float
    acquisitions: tuple[Acquisition, ...]
    transport_uncertainty_s: float | None
    source_progression_verified: bool
    calibration_digest: str
    velocity_source: str
    registration_valid_until: float
    fault: str | None = None
    position_uncertainty_m: float | None = None
    velocity_uncertainty_rad_s: float | None = None

    def __post_init__(self):
        object.__setattr__(self, 'acquisitions', tuple(self.acquisitions))

    def validate(self, now, limits, *, require_registration=True):
        for field in ('bundle_id', 'boot_id', 'owner_epoch', 'calibration_digest', 'velocity_source'):
            text(getattr(self, field), field)
        if type(self.sequence) is not int or self.sequence < 0 or self.fault:
            raise RuntimeError('Invalid/faulted observation bundle')
        if self.source_progression_verified is not True or self.transport_uncertainty_s is None:
            raise RuntimeError('Acquisition progression/transport uncertainty is uncommissioned')
        for name in ('position_uncertainty_m', 'velocity_uncertainty_rad_s'):
            bound = getattr(self, name)
            if bound is None or finite(bound, name) < 0:
                raise RuntimeError('Measurement uncertainty is uncommissioned')
        uncertainty = finite(self.transport_uncertainty_s, 'transport uncertainty')
        if uncertainty < 0 or uncertainty > limits['max_transport_uncertainty_s']:
            raise RuntimeError('Transport uncertainty exceeds approved bound')
        if {value.name for value in self.acquisitions} != {'joints', 'identity', 'status', 'left_hand', 'right_hand'} or len(self.acquisitions) != 5:
            raise RuntimeError('Observation bundle components incomplete/duplicated')
        for value in self.acquisitions:
            value.validate()
        oldest = min(value.earliest for value in self.acquisitions)
        newest = max(value.latest for value in self.acquisitions)
        if (not finite(self.published_at, 'publication') <= now
                or (require_registration and now >= finite(self.registration_valid_until, 'registration expiry'))
                or newest > self.published_at or now-oldest > limits['max_age_s']
                or newest-oldest > limits['max_skew_s']):
            raise RuntimeError('Observation age/skew/registration outside approved bounds')


class LatestSampleCache:
    """A read never calls SDK code. The sole sampler publishes detached samples.

    A blocked or dead sampler cannot block readers or renew cached timestamps.
    This is the in-process receiver for an independently supervised observation
    process, not an independent stop watchdog or proof of source progression.
    """
    def __init__(self, *, clock, max_age_s):
        self.clock, self.max_age_s = clock, finite(max_age_s, 'cache age', positive=True)
        self._lock = threading.Lock()
        self._sample = None
        self._epoch = None
        self._sequence = -1
        self._fault = None

    def publish(self, sample):
        bundle = getattr(sample, 'bundle', None)
        if not isinstance(bundle, ObservationBundle):
            raise ValueError('Sampler must publish an immutable evidence bundle')
        with self._lock:
            if self._epoch is not None and bundle.boot_id != self._epoch:
                self._fault = 'Sampler boot changed; explicit revalidation required'
                raise RuntimeError(self._fault)
            if bundle.sequence <= self._sequence:
                raise RuntimeError('Duplicate/regressing sampler sequence')
            self._epoch, self._sequence, self._sample = bundle.boot_id, bundle.sequence, sample

    def read(self):
        with self._lock:
            sample, fault = self._sample, self._fault
        if fault or sample is None:
            raise RuntimeError(fault or 'No observation sample')
        if not 0 <= self.clock()-sample.bundle.published_at <= self.max_age_s:
            raise RuntimeError('Observation sampler stopped or cache expired')
        return sample
