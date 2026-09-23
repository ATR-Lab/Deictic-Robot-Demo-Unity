"""Bounded wall-time scheduling for physics with a slower synchronous renderer."""
import math
import time


class RealtimeSchedule:
    """Catch up using observed simulation time, never a guessed step multiplier.

    Physics keeps its fixed authored dt. Rendering is due on wall time but waits
    for a bounded catch-up burst, so a costly render cannot reduce every second
    of actuator motion to only a handful of physics steps. Long pauses discard
    excess debt instead of replaying seconds of motion or starving rendering.

    When catch-up cannot clear the lag, a render gets its next turn no later
    than one render period after the previous render finished, at the next
    scheduling boundary. A render costing one period therefore has at most
    two periods between starts. Measuring this bound from completion leaves
    physics a catch-up opportunity even when rendering itself exceeds it.
    """
    def __init__(self, wall_start, simulation_start, physics_dt=1/120., render_hz=30.,
                 max_debt_s=.25, max_steps_without_render=32):
        if (not all(math.isfinite(value) for value in
                    (wall_start, simulation_start, physics_dt, render_hz, max_debt_s))
                or physics_dt <= 0 or render_hz <= 0 or max_debt_s < physics_dt
                or type(max_steps_without_render) is not int or max_steps_without_render < 1):
            raise ValueError("Invalid real-time physics/render schedule")
        self.wall_start = self.wall_anchor = wall_start
        self.simulation_start = simulation_start
        self.physics_dt = physics_dt
        self.render_period = 1./render_hz
        self.max_debt_s = max_debt_s
        self.max_steps_without_render = max_steps_without_render
        self.next_render = wall_start
        self.render_fairness_deadline = wall_start+self.render_period
        self.steps_since_render = self.physics_iterations = self.render_calls = 0
        self.dropped_debt_s = self.max_observed_lag_s = 0.
        self.last_render_s = 0.

    def plan(self, now, simulation_time):
        """Return (physics_due, render_after_step, bounded_sleep_seconds)."""
        if not math.isfinite(now) or not math.isfinite(simulation_time):
            raise ValueError("Schedule clocks must be finite")
        lag = now-self.wall_anchor-(simulation_time-self.simulation_start)
        self.max_observed_lag_s = max(self.max_observed_lag_s, lag)
        if lag > self.max_debt_s:
            dropped = lag-self.max_debt_s
            self.wall_anchor += dropped
            self.dropped_debt_s += dropped
            lag = self.max_debt_s
        if lag < -1e-6:
            return False, False, min(self.physics_dt, -lag)
        render = now >= self.next_render and (lag <= self.physics_dt+1e-6
                    or self.steps_since_render >= self.max_steps_without_render
                    or now >= self.render_fairness_deadline)
        return True, render, 0.

    def completed(self, started, finished, rendered):
        self.physics_iterations += 1
        self.steps_since_render += 1
        if rendered:
            self.render_calls += 1
            self.last_render_s = max(0., finished-started)
            self.next_render = started+self.render_period
            self.render_fairness_deadline = finished+self.render_period
            self.steps_since_render = 0

    def diagnostic(self, now, simulation_time):
        elapsed = max(0., now-self.wall_start)
        simulated = max(0., simulation_time-self.simulation_start)
        return dict(wall_elapsed_s=elapsed, simulation_elapsed_s=simulated,
                    real_time_factor=simulated/elapsed if elapsed > 0 else None,
                    dropped_wall_debt_s=self.dropped_debt_s,
                    max_observed_physics_lag_s=self.max_observed_lag_s,
                    physics_iterations=self.physics_iterations, render_calls=self.render_calls,
                    last_render_step_s=self.last_render_s,
                    max_render_catchup_s=self.render_period)


def drain_callbacks(spin_once, max_callbacks=4, budget_s=.002, monotonic=time.monotonic):
    """Give both depth-one actuator topics a chance before physics, without waiting."""
    started = monotonic()
    count = 0
    while count < max_callbacks:
        spin_once(timeout_sec=0)
        count += 1
        if monotonic()-started >= budget_s:
            break
    return count
