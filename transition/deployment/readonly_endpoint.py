#!/usr/bin/env python3
"""ROS-TCP display endpoint: refuse ALL Unity publications and RPC routes."""
from pathlib import Path
import sys


def install_readonly(commands):
    def deny(self, *args, **kwargs):
        self.tcp_server.send_unity_error("Transition endpoint is receive-only: publications and services are disabled")
    for name in ("publish", "ros_service", "unity_service", "request", "response"):
        setattr(commands, name, deny)


def main():
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"ros2/scripts"))
    from ros_tcp_endpoint.server import SysCommands
    from run_endpoint import main as endpoint_main
    install_readonly(SysCommands)
    endpoint_main()

if __name__ == "__main__": main()
