"""Isolated launcher checks; no ROS, Docker daemon, GPU, SSH or robot required.

Run on Ubuntu/WSL: /usr/bin/python3 -m unittest discover -s scripts -p test_start_demo_remote.py
"""

import contextlib
import importlib.util
import io
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock


spec = importlib.util.spec_from_file_location("start_demo_remote", Path(__file__).with_name("start_demo_remote.py"))
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class PlanTests(unittest.TestCase):
    def test_dry_run_does_not_probe_or_create_paths(self):
        with tempfile.TemporaryDirectory() as parent:
            repo = Path(parent) / "does-not-exist"
            with mock.patch.object(launcher, "preflight", side_effect=AssertionError), \
                    mock.patch.object(launcher, "sourced_environment", side_effect=AssertionError), \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(0, launcher.main(["--repo", str(repo), "--public-ip",
                                                   "131.123.237.31", "--dry-run"]))
            self.assertFalse(repo.exists())
            text = output.getvalue()
            self.assertIn("--camera-mount-profile reach-projection-balanced", text)
            self.assertIn("/usr/bin/python3 sim/trajectory_relay.py", text)
            self.assertIn("run_bridge.sh markerless", text)
            self.assertIn(".venv-registration/bin/python", text)
            self.assertIn("DEICTIC_K1_READY", text)

    def test_port_conflict_closes_probes_without_stopping_processes(self):
        fake_socket = mock.Mock()
        fake_socket.bind.side_effect = OSError(98, "Address already in use")
        with mock.patch.object(launcher.socket, "socket", return_value=fake_socket), \
                mock.patch.object(launcher.subprocess, "run") as run:
            with self.assertRaisesRegex(RuntimeError, "TCP 0.0.0.0:10000 is unavailable"):
                launcher.probe_ports()
            fake_socket.close.assert_called_once()
            run.assert_not_called()

    def test_partial_source_upgrade_fails_before_probing_or_starting(self):
        for omitted in ("sim/head_control.py", "sim/head_stereo.py",
                        "sim/loop_timing.py", "ros2/scripts/run_endpoint.py"):
            with self.subTest(missing=omitted), tempfile.TemporaryDirectory() as parent:
                repo = Path(parent)
                for relative in (*launcher.REQUIRED_SOURCES, ".venv-control/bin/python",
                                 ".venv-registration/bin/python"):
                    if relative == omitted:
                        continue
                    path = repo / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.touch()
                (repo / ".codex/ros2/vendor/ROS-TCP-Endpoint").mkdir(parents=True)
                with mock.patch.object(launcher, "probe_ports") as ports, \
                        mock.patch.object(launcher, "run_checked") as run:
                    with self.assertRaises(RuntimeError) as error:
                        launcher.preflight(repo, {"PATH": ""})
                self.assertIn(omitted, str(error.exception).replace("\\", "/"))
                self.assertIn("transfers only itself", str(error.exception))
                ports.assert_not_called()
                run.assert_not_called()


@unittest.skipUnless(sys.platform == "linux", "Process-group ownership requires Ubuntu/WSL")
class OwnershipTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.repo = Path(self.temp.name)
        self.supervisor = launcher.Supervisor(self.repo, dict(os.environ), self.repo)
        self.handlers = {sig: signal.getsignal(sig) for sig in
                         (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}

    def tearDown(self):
        self.supervisor.cleanup()
        for sig, handler in self.handlers.items():
            signal.signal(sig, handler)
        self.temp.cleanup()

    def wait_file(self, path, timeout=5):
        deadline = time.monotonic() + timeout
        while not path.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(path.exists(), f"Child did not create {path}")

    def test_cleanup_signals_owned_descendants_but_preserves_unrelated_job(self):
        completed = self.repo / "child-stopped"
        ready = self.repo / "ready"
        child_code = (
            "import signal,time,pathlib\n"
            f"done=pathlib.Path({str(completed)!r})\n"
            "def stop(*_):\n    done.write_text('SIGINT')\n    raise SystemExit(0)\n"
            "signal.signal(signal.SIGINT,stop)\n"
            f"pathlib.Path({str(ready)!r}).write_text('ready')\n"
            "while True: time.sleep(.05)\n")
        parent_code = (
            "import subprocess,sys\n"
            f"child=subprocess.Popen([sys.executable,'-c',{child_code!r}])\n"
            "try: child.wait()\n"
            "except KeyboardInterrupt: child.wait()\n")
        unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                                     start_new_session=True)
        try:
            owned = self.supervisor.start("relay", [sys.executable, "-c", parent_code])
            self.wait_file(ready)
            self.supervisor.cleanup()
            self.assertIsNotNone(owned.poll())
            self.assertEqual("SIGINT", completed.read_text())
            self.assertIsNone(unrelated.poll())
        finally:
            unrelated.terminate()
            unrelated.wait(timeout=5)

    def test_container_cleanup_uses_only_exact_owned_id(self):
        self.supervisor.container_name = "deictic-k1-session-test-123"
        container_id = "a" * 64
        inspect = subprocess.CompletedProcess([], 0, stdout=container_id + "\n")
        stop = subprocess.CompletedProcess([], 0)
        with mock.patch.object(launcher.subprocess, "run", side_effect=[inspect, stop]) as run:
            self.supervisor.cleanup()
        self.assertEqual(
            ["docker", "inspect", "--type", "container", "--format", "{{.Id}}",
             "deictic-k1-session-test-123"], run.call_args_list[0].args[0])
        self.assertEqual(["docker", "stop", "--time", "10", container_id],
                         run.call_args_list[1].args[0])

    def test_missing_owned_container_does_not_stop_anything(self):
        self.supervisor.container_name = "deictic-k1-session-missing-321"
        with mock.patch.object(launcher.subprocess, "run", return_value=
                               subprocess.CompletedProcess([], 1, stdout="")) as run:
            self.supervisor.cleanup()
        self.assertEqual(1, run.call_count)
        self.assertEqual("inspect", run.call_args.args[0][1])

    def test_child_failure_reports_log_and_never_launches_following_service(self):
        child = self.supervisor.start("isaac", [sys.executable, "-c",
                                                "print('fixture startup failure'); raise SystemExit(7)"])
        child.wait(timeout=5)
        with self.assertRaisesRegex(RuntimeError, r"isaac exited \(7\).*[\s\S]*fixture startup failure"):
            self.supervisor.wait_for_isaac(1)
        self.assertEqual(["isaac"], [name for name, _, _ in self.supervisor.children])
        # This fixture is a Python subprocess, never a Docker container.
        self.supervisor.container_name = None

    def test_readiness_wait_times_out_without_starting_later_services(self):
        self.supervisor.start("isaac", [sys.executable, "-c", "import time; time.sleep(30)"])
        self.supervisor.container_name = None
        with self.assertRaisesRegex(RuntimeError, "did not report DEICTIC_K1_READY"):
            self.supervisor.wait_for_isaac(0.05)
        self.assertEqual(1, len(self.supervisor.children))

    def test_readiness_marker_allows_next_stage(self):
        self.supervisor.start("isaac", [sys.executable, "-u", "-c",
                                       "import time; print('DEICTIC_K1_READY'); time.sleep(30)"])
        self.supervisor.container_name = None
        self.supervisor.wait_for_isaac(5)


if __name__ == "__main__":
    unittest.main()
