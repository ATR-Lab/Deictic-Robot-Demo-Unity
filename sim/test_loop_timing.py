"""Deterministic lag/callback checks without Isaac, ROS or real-time sleeps."""
import unittest

from loop_timing import RealtimeSchedule, drain_callbacks


class RealtimeScheduleTests(unittest.TestCase):
    def test_observed_time_prevents_assuming_render_steps_equal_one_physics_dt(self):
        schedule = RealtimeSchedule(10., 5.)
        self.assertEqual(schedule.plan(10., 5.), (True, True, 0.))
        schedule.completed(10., 10.001, True)
        # A runtime advanced four physics dt during one render-capable step.
        due, render, delay = schedule.plan(10.001, 5.+4/120.)
        self.assertFalse(due)
        self.assertFalse(render)
        self.assertAlmostEqual(delay, 1/120.)
        self.assertTrue(schedule.plan(10.+4/120., 5.+4/120.)[0])

    def test_renderer_stall_is_followed_by_nonrendering_physics_catchup(self):
        schedule = RealtimeSchedule(0., 0.)
        schedule.completed(0., .15, True)
        self.assertEqual(schedule.plan(.15, 1/120.), (True, False, 0.))
        self.assertEqual(schedule.plan(.15, .15), (True, True, 0.))

    def test_long_pause_drops_excess_debt_and_bounds_render_starvation(self):
        schedule = RealtimeSchedule(0., 0., max_debt_s=.25, max_steps_without_render=4)
        self.assertEqual(schedule.plan(60., 0.), (True, True, 0.))
        self.assertAlmostEqual(schedule.dropped_debt_s, 59.75)
        schedule.completed(60., 60.02, True)
        # Even if the physics engine does not advance, rendering gets a turn.
        for _ in range(4):
            schedule.completed(60., 60., False)
        self.assertTrue(schedule.plan(60.04, 0.)[1])

    def test_persistent_lag_cannot_postpone_render_until_32_physics_steps(self):
        schedule = RealtimeSchedule(0., 0.)
        schedule.completed(0., .034, True)
        self.assertFalse(schedule.plan(.05, .01)[1])
        self.assertLess(schedule.steps_since_render, schedule.max_steps_without_render)
        # A busy control loop has not caught up, but the wall-time bound expires
        # one render period after completion (~67 ms between 34 ms render starts).
        self.assertTrue(schedule.plan(.034+1/30., .02)[1])

    def test_slow_render_still_leaves_physics_a_catchup_interval(self):
        schedule = RealtimeSchedule(0., 0.)
        schedule.completed(0., .15, True)
        self.assertFalse(schedule.plan(.15, 1/120.)[1])
        self.assertFalse(schedule.plan(.17, 2/120.)[1])
        self.assertTrue(schedule.plan(.15+1/30., 3/120.)[1])

    def test_timing_diagnostics_use_observed_simulation_clock(self):
        schedule = RealtimeSchedule(10., 100.)
        schedule.completed(10., 10.1, True)
        stats = schedule.diagnostic(12., 101.8)
        self.assertAlmostEqual(stats['real_time_factor'], .9)
        self.assertEqual(stats['render_calls'], 1)
        self.assertAlmostEqual(stats['last_render_step_s'], .1)

    def test_150ms_renderer_does_not_force_physics_to_its_frame_rate(self):
        schedule = RealtimeSchedule(0., 0.)
        wall = simulated = 0.
        while wall < 3.:
            due, render, sleep = schedule.plan(wall, simulated)
            if not due:
                wall += sleep
                continue
            started = wall
            simulated += 1/120.
            wall += .0005 + (.15 if render else 0.)
            schedule.completed(started, wall, render)
        self.assertGreater(simulated/wall, .90)
        self.assertEqual(schedule.dropped_debt_s, 0.)
        self.assertGreater(schedule.physics_iterations, schedule.render_calls*10)

    def test_invalid_schedule_is_rejected(self):
        for values in ({'physics_dt': 0}, {'render_hz': 0}, {'max_debt_s': 0},
                       {'max_steps_without_render': 0}, {'wall_start': float('nan')}):
            arguments = dict(wall_start=0., simulation_start=0.); arguments.update(values)
            with self.subTest(values=values), self.assertRaises(ValueError):
                RealtimeSchedule(**arguments)


class CallbackDrainTests(unittest.TestCase):
    def test_both_command_queues_are_serviced_without_an_unbounded_drain(self):
        calls = []
        count = drain_callbacks(lambda **kwargs: calls.append(kwargs), monotonic=lambda: 0.)
        self.assertEqual(count, 4)
        self.assertEqual(calls, [{'timeout_sec': 0}]*4)

    def test_expensive_callback_exhausts_budget_before_more_work(self):
        clock = iter([0., .003])
        calls = []
        self.assertEqual(drain_callbacks(lambda **kwargs: calls.append(kwargs), monotonic=lambda: next(clock)), 1)


if __name__ == '__main__':
    unittest.main()
