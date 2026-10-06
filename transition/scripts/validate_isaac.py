#!/usr/bin/env python3
"""Exercise a running native Isaac worker and save actual measurement receipts."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from transition_autonomy.backends.base import SkillCommand
from transition_autonomy.backends.isaac_backend import IsaacBackend


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--endpoint", default="http://127.0.0.1:8767")
    parser.add_argument("--output", default="artifacts/isaac/validation_receipt.json")
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve writable evidence before the first simulated dispatch. Never replace
    # an earlier failed attempt to make a later run look like the first attempt.
    with output.open("x") as stream:
        stream.write('{"passed":false,"status":"started"}\n')
    backend = IsaacBackend(args.endpoint)
    receipt = {"test": "native Isaac fixed-support K1 command lifecycle", "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "commands": []}

    def wait_quiescent():
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            obs = backend.observe(time.monotonic())
            if obs.connected and obs.quiescent:
                return obs
            time.sleep(.05)
        raise RuntimeError(f"worker did not settle: {obs}")

    try:
        receipt["initial_observation"] = asdict(wait_quiescent())
        for profile, expected in [("point_a", "A"), ("point_b", "B"), ("home", None)]:
            now = time.monotonic()
            command = SkillCommand(str(uuid.uuid4()), "native-validation", profile, "point", {"profile": profile}, {}, "validation", 1, now, now+25)
            accepted = backend.start(command, now)
            assert accepted.status == "accepted", accepted
            duplicate = backend.start(command, time.monotonic())
            assert duplicate.status in {"accepted", "running"}, duplicate
            entry = {"command": command.to_dict(), "accepted": asdict(accepted), "duplicate": asdict(duplicate)}
            receipt["commands"].append(entry)
            while time.monotonic() < now + 28:
                event = backend.command_status(command.command_id, time.monotonic())
                if event.status in {"succeeded", "failed", "canceled", "unknown"}:
                    break
                time.sleep(.05)
            entry["terminal"] = asdict(event)
            entry["wall_duration_seconds"] = time.monotonic()-now
            assert event.status == "succeeded", event
            assert event.facts["remote.pointed_target"].value == expected
            assert event.evidence["settle_clock"] == "advancing_physics_simulation_time"
            assert event.evidence["velocity_source"] == "position_difference_per_sim_second"
            assert event.evidence["settle_seconds"] >= .3
            entry["settled_observation"] = asdict(wait_quiescent())
            frame = Path("artifacts/isaac/latest_frame.png")
            time.sleep(2.2)
            if frame.exists():
                import shutil
                shutil.copyfile(frame, frame.with_name(profile + ".png"))
        now = time.monotonic()
        command = SkillCommand(str(uuid.uuid4()), "native-validation", "stop-probe", "point", {"profile": "point_a"}, {}, "validation", 1, now, now+25)
        receipt["stop_accepted"] = asdict(backend.start(command, now))
        time.sleep(.5)
        backend.request_stop(time.monotonic())
        receipt["stop_observation"] = asdict(wait_quiescent())
        receipt["stop_terminal"] = asdict(backend.command_status(command.command_id, time.monotonic()))
        assert receipt["stop_terminal"]["status"] == "canceled"
        assert receipt["stop_observation"]["facts"]["remote.pointed_target"]["status"] == "unknown"
        # Actual loopback TCP response-loss injection: the worker accepts a
        # command, but a proxy closes the client socket without its response.
        # Reconnection queries that same command; no new dispatch is needed.
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        import threading
        import urllib.request
        link = {"drop": False}
        class LossProxy(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def forward(self):
                if link["drop"]:
                    self.close_connection = True
                    return
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length) if self.command == "POST" else None
                request = urllib.request.Request(args.endpoint + self.path, data=body,
                                                  headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=2) as response:
                    raw = response.read()
                if self.path == "/commands/start":
                    link["drop"] = True
                    self.close_connection = True
                    return
                self.send_response(200)
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            do_POST = forward
            do_GET = forward
        proxy = ThreadingHTTPServer(("127.0.0.1", 0), LossProxy)
        threading.Thread(target=proxy.serve_forever, daemon=True).start()
        uncertain = IsaacBackend(f"http://127.0.0.1:{proxy.server_port}")
        now = time.monotonic()
        command = SkillCommand(str(uuid.uuid4()), "native-validation", "response-loss-probe", "point", {"profile": "home"}, {}, "validation", 1, now, now+25)
        try:
            receipt["response_loss_injection"] = {"method": "loopback TCP proxy drops accepted dispatch response and subsequent reads",
                "command": command.to_dict(), "dispatch": asdict(uncertain.start(command, now))}
            lost = receipt["response_loss_injection"]
            assert lost["dispatch"]["status"] == "unknown"
            lost["disconnected_observation"] = asdict(uncertain.observe(time.monotonic()))
            assert not lost["disconnected_observation"]["connected"]
            assert not lost["disconnected_observation"]["quiescent"]
            assert not lost["disconnected_observation"]["facts"]
            time.sleep(.3)
            link["drop"] = False
            while time.monotonic() < now+28:
                event = uncertain.command_status(command.command_id, time.monotonic())
                if event.status in {"succeeded", "failed", "canceled", "unknown"}:
                    break
                time.sleep(.05)
            lost["reconciled_terminal"] = asdict(event)
            assert event.status == "succeeded"
            assert event.facts["remote.pointed_target"].value is None
            lost["settled_observation"] = asdict(wait_quiescent())
        finally:
            proxy.shutdown()
            proxy.server_close()
        receipt["passed"] = True
    except Exception as exc:
        receipt.update(passed=False, error=repr(exc))
        raise
    finally:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(json.dumps(receipt, indent=2) + "\n")
        print(json.dumps({"passed": receipt.get("passed"), "output": args.output, "error": receipt.get("error")}))


if __name__ == "__main__":
    main()
