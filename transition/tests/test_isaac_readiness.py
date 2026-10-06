"""Startup readiness must not mistake an open HTTP port for settled physics."""
import pytest

from transition_autonomy.isaac_readiness import ReadinessGate, observation_url, wait_until_ready


def observation(stamp, *, boot="boot-1", **changes):
    result = {"boot_id": boot, "server_now": stamp + .01,
              "observation": {"observed_at": stamp, "quiescent": True,
                              "active_command_id": None, "fault": None}}
    result["observation"].update(changes)
    return result


def test_initial_settling_requires_fresh_continuous_window():
    gate = ReadinessGate()
    assert not gate.sample({}, 0)
    assert not gate.sample(observation(10, fault="starting"), .1)
    assert not gate.sample(observation(10.1, quiescent=False), .2)
    assert not gate.sample(observation(10.2, active_command_id="other"), .3)
    assert not gate.sample(observation(10.3), .4)
    assert not gate.sample(observation(10.5), .6)
    assert gate.sample(observation(10.7), .8)


@pytest.mark.parametrize("change", [
    {"observed_at": float("nan")}, {"observed_at": float("inf")},
    {"observed_at": True}, {"observed_at": 11}, {"observed_at": 9},
    {"quiescent": 1}, {"quiescent": "true"}, {"fault": "starting"},
    {"active_command_id": "owned-by-another-run"},
])
def test_invalid_or_nonidle_evidence_resets_accumulated_window(change):
    gate = ReadinessGate()
    assert not gate.sample(observation(9.8), 0)
    assert not gate.sample(observation(10, **change), .2)
    assert not gate.sample(observation(10.2), .4)
    assert gate.sample(observation(10.6), .8)


@pytest.mark.parametrize("field,value", [("boot_id", ""), ("boot_id", None),
                                        ("server_now", float("nan")),
                                        ("server_now", True)])
def test_invalid_envelope_cannot_be_ready(field, value):
    gate = ReadinessGate()
    payload = observation(10)
    payload[field] = value
    assert not gate.sample(payload, 1)


@pytest.mark.parametrize("field", ["fault", "active_command_id"])
def test_missing_owner_or_fault_status_is_not_proof_of_readiness(field):
    gate = ReadinessGate()
    payload = observation(10)
    del payload["observation"][field]
    assert not gate.sample(payload, 1)


def test_repeated_snapshot_cannot_count_as_advancing_settled_time():
    gate = ReadinessGate()
    for received in (0, .2, .4, .6):
        payload = observation(10)
        payload["server_now"] = 10 + received
        assert not gate.sample(payload, received)


def test_boot_change_or_acquisition_gap_starts_a_new_window():
    gate = ReadinessGate()
    assert not gate.sample(observation(10), 0)
    assert not gate.sample(observation(10.2), .2)
    assert not gate.sample(observation(10.4, boot="boot-2"), .4)
    assert not gate.sample(observation(11, boot="boot-2"), 1)
    assert gate.sample(observation(11.4, boot="boot-2"), 1.4)


def test_both_source_and_local_time_must_advance():
    gate = ReadinessGate()
    assert not gate.sample(observation(10), 0)
    assert not gate.sample(observation(10.4), .1)
    assert gate.sample(observation(10.5), .4)


def test_round_trip_counts_against_freshness_budget():
    gate = ReadinessGate()
    assert not gate.sample(observation(10), 0)
    assert not gate.sample(observation(10.4), .4, round_trip=.5)
    assert "stale" in gate.reason


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_polling_waits_through_connection_and_initial_settling_without_commands():
    clock = FakeClock()
    routes = []

    def fetch(url, timeout):
        routes.append(url)
        if clock.now < .1:
            raise OSError("not listening")
        return observation(10 + clock.now, quiescent=clock.now >= .2)

    result = wait_until_ready(timeout=2, fetch=fetch, clock=clock, sleep=clock.sleep)
    assert result["observation"]["quiescent"]
    assert clock.now >= .5
    assert routes and set(routes) == {"http://127.0.0.1:8767/observation"}


def test_polling_timeout_retains_reason_and_does_not_dispatch():
    clock = FakeClock()
    with pytest.raises(TimeoutError, match="has not measured quiescence"):
        wait_until_ready(timeout=.4, fetch=lambda url, timeout: observation(clock.now, quiescent=False),
                         clock=clock, sleep=clock.sleep)


@pytest.mark.parametrize("endpoint", ["http://192.168.10.102:8767", "https://127.0.0.1:8767",
                                       "http://127.0.0.1:8767/commands/start",
                                       "http://user:password@127.0.0.1:8767",
                                       "http://127.0.0.1:8767?x=y", "http://127.0.0.1:0"])
def test_nonloopback_or_unexpected_endpoint_refused_before_fetch(endpoint):
    def unexpected_fetch(*args):
        pytest.fail("invalid endpoint must not contact a server")
    with pytest.raises(ValueError, match="HTTP loopback"):
        wait_until_ready(endpoint, fetch=unexpected_fetch)


def test_loopback_endpoint_resolution():
    assert observation_url("http://127.0.0.1:8767/") == "http://127.0.0.1:8767/observation"
    assert observation_url("http://[::1]:8767") == "http://[::1]:8767/observation"
