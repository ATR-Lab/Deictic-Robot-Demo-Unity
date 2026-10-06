"""Exercise container ownership cleanup without Docker, ROS, or a simulator."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


pytestmark = pytest.mark.skipif(os.name != "posix", reason="Linux deployment launcher")
SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "run_isaac_worker.sh"


@pytest.fixture
def launcher(tmp_path):
    script = tmp_path / "project" / "transition" / "scripts" / SOURCE.name
    script.parent.mkdir(parents=True)
    script.write_text(SOURCE.read_text())
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text('''#!/usr/bin/python3
import json,os,pathlib,sys,time
args=sys.argv[1:]
with open(os.environ["FAKE_LOG"],"a") as stream:
    stream.write(json.dumps(args)+"\\n")
owner=pathlib.Path(os.environ["FAKE_OWNER"])
mode=os.environ.get("FAKE_MODE", "normal")
if args[0]=="run":
    if mode=="name_collision": sys.exit(125)
    pathlib.Path(args[args.index("--cidfile")+1]).write_text("a"*64)
    owner.write_text(args[args.index("--label")+1].split("=",1)[1])
elif args[0]=="inspect":
    print("somebody-else" if mode=="owner_mismatch" else owner.read_text())
elif args[0]=="stop":
    pathlib.Path(os.environ["FAKE_STOP"]).write_text("started")
    time.sleep(.4)
''')
    docker.chmod(0o755)
    env = dict(os.environ, PATH=str(fake_bin) + os.pathsep + os.environ["PATH"],
               FAKE_LOG=str(tmp_path / "docker.jsonl"), FAKE_OWNER=str(tmp_path / "owner"),
               FAKE_STOP=str(tmp_path / "stopping"))
    return script, env


def calls(env):
    path = Path(env["FAKE_LOG"])
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def test_owned_cleanup_finishes_despite_reentrant_term(launcher):
    script, env = launcher
    process = subprocess.Popen(["bash", str(script)], env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 5
        while not Path(env["FAKE_STOP"]).exists():
            assert process.poll() is None, process.communicate()
            assert time.monotonic() < deadline
            time.sleep(.01)
        process.send_signal(signal.SIGTERM)
        _, error = process.communicate(timeout=5)
        assert process.returncode == 0, error
        commands = calls(env)
        assert [entry[0] for entry in commands] == ["run", "inspect", "stop", "rm"]
        assert commands[-1] == ["rm", "--force", "a" * 64]
        assert commands[-2][-1] == "a" * 64
        assert "--network" in commands[0] and "ROS_DOMAIN_ID=174" in commands[0]
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


@pytest.mark.parametrize("mode,expected", [("name_collision", ["run"]),
                                          ("owner_mismatch", ["run", "inspect"])])
def test_never_cleans_preexisting_or_mismatched_container(launcher, mode, expected):
    script, env = launcher
    env["FAKE_MODE"] = mode
    result = subprocess.run(["bash", str(script)], env=env, capture_output=True, text=True, timeout=5)
    assert result.returncode == (125 if mode == "name_collision" else 0)
    assert [entry[0] for entry in calls(env)] == expected


def test_output_escape_refused_before_container_creation(launcher):
    script, env = launcher
    result = subprocess.run(["bash", str(script), "--output", "../../outside"], env=env,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode != 0
    assert "must remain within" in result.stderr
    assert calls(env) == []
