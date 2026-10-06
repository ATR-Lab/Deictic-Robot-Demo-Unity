"""Linux service and optional WebRTC socket preflight.

This is a startup diagnostic, not a port reservation: the real servers still
handle bind errors if another process starts between this check and startup.
No processes are contacted, adopted, or stopped.
"""
from __future__ import annotations

import argparse
import errno
import socket
import sys


STACK_PORTS = {
    "logical": (10000, 8766),
    "isaac": (10000, 8766, 8767),
    "hardware-observation": (10000,),
}
STREAM_PORTS = (("TCP", 49100), ("UDP", 47998))
OPTIONAL_IPV6_ERRORS = (errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL)


class PortPreflightError(RuntimeError):
    def __init__(self, port: int, error: OSError, *, transport: str = "TCP",
                 address: str = "127.0.0.1"):
        self.port = port
        self.error = error
        self.transport = transport
        self.address = address
        if address == "127.0.0.1":
            endpoint = f"workstation loopback {address}:{port}"
        else:
            host = f"[{address}]" if ":" in address else address
            endpoint = f"workstation WebRTC {host}:{port}"
        operation = "listen" if transport == "TCP" else "bind"
        super().__init__(f"Cannot {operation} on {endpoint} ({transport}): {error}")


def check_port_available(port: int) -> None:
    """Allow reuse after TIME_WAIT, while refusing an existing TCP listener."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", port))
            # Two sockets with REUSEADDR may both bind before either listens.
            # Reaching listen is the relevant condition for starting our server.
            probe.listen(1)
    except OSError as error:
        raise PortPreflightError(port, error) from error


def check_stream_port_available(transport: str, port: int) -> None:
    """Reject an existing IPv4 or IPv6 stream endpoint without contacting it.

    TCP uses REUSEADDR plus listen to permit a closed server's TIME_WAIT.
    UDP has neither reuse option, including when an existing owner uses one.
    IPv6 is optional, but a conflict or permission error is never ignored.
    """
    if transport not in ("TCP", "UDP"):
        raise ValueError("stream transport must be TCP or UDP")
    kind = socket.SOCK_STREAM if transport == "TCP" else socket.SOCK_DGRAM
    for family, address in ((socket.AF_INET, "0.0.0.0"), (socket.AF_INET6, "::")):
        try:
            with socket.socket(family, kind) as probe:
                if transport == "TCP":
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                if family == socket.AF_INET6:
                    probe.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                probe.bind((address, port))
                if transport == "TCP":
                    probe.listen(1)
        except OSError as error:
            if family == socket.AF_INET6 and error.errno in OPTIONAL_IPV6_ERRORS:
                continue
            raise PortPreflightError(port, error, transport=transport, address=address) from error


def failure_message(error: PortPreflightError) -> str:
    lines = [str(error), "No services started and nothing was stopped."]
    if error.error.errno == errno.EADDRINUSE:
        inspect_flags = "-ltnp" if error.transport == "TCP" else "-aunp"
        lines.extend([
            f"Inspect the {error.transport} endpoint on the workstation (not the Windows host):",
            f"  ss {inspect_flags} 'sport = :{error.port}'",
            "If it belongs to an earlier intended launch, close that launch's terminal first.",
            "If no listener remains, retry startup; the conflicting process may have exited.",
        ])
    else:
        lines.append("Resolve the socket error above before retrying startup.")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=STACK_PORTS)
    parser.add_argument("--webrtc", action="store_true", help="Check Isaac streaming TCP/UDP ports too")
    args = parser.parse_args(argv)
    if args.webrtc and args.mode != "isaac":
        parser.error("--webrtc is supported only in isaac mode")
    try:
        for port in STACK_PORTS[args.mode]:
            check_port_available(port)
        if args.webrtc:
            for transport, port in STREAM_PORTS:
                check_stream_port_available(transport, port)
    except PortPreflightError as error:
        print(failure_message(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
