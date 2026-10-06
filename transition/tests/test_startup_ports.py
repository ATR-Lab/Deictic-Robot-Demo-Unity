"""Real Linux TCP sockets: startup can reuse closed connections, never listeners."""
import errno
import socket
import sys

import pytest

from transition_autonomy.startup_ports import (
    PortPreflightError, check_port_available, check_stream_port_available, failure_message, main,
)


@pytest.mark.skipif(sys.platform != "linux", reason="Ubuntu stack uses Linux TCP reuse semantics")
@pytest.mark.parametrize("address", ["127.0.0.1", "0.0.0.0"])
@pytest.mark.parametrize("reuse_address", [False, True])
def test_live_listener_is_refused_and_remains_usable(address, reuse_address):
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, int(reuse_address))
        listener.bind((address, 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        with pytest.raises(PortPreflightError) as caught:
            check_port_available(port)
        assert caught.value.error.errno == errno.EADDRINUSE
        with socket.create_connection(("127.0.0.1", port), timeout=1) as client:
            connection, _ = listener.accept()
            with connection:
                connection.sendall(b"still owned by original listener")
                assert client.recv(100) == b"still owned by original listener"


@pytest.mark.skipif(sys.platform != "linux", reason="Ubuntu stack uses Linux TCP reuse semantics")
def test_restart_allows_closed_server_connection_in_time_wait():
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        with socket.create_connection(("127.0.0.1", port), timeout=1) as client:
            connection, _ = listener.accept()
            # Server sends the first FIN, so its local port retains TIME_WAIT.
            connection.close()
            assert client.recv(1) == b""
    with socket.socket() as old_probe:
        with pytest.raises(OSError) as caught:
            old_probe.bind(("127.0.0.1", port))
        assert caught.value.errno == errno.EADDRINUSE
    check_port_available(port)
    check_stream_port_available("TCP", port)
    # The preflight closes its temporary listener; the service can bind next.
    with socket.socket() as restarted:
        restarted.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        restarted.bind(("127.0.0.1", port))
        restarted.listen(1)


def test_address_in_use_message_identifies_remote_inspection_without_killing():
    message = failure_message(PortPreflightError(8766, OSError(errno.EADDRINUSE, "Address already in use")))
    assert "workstation loopback 127.0.0.1:8766" in message
    assert "ss -ltnp 'sport = :8766'" in message
    assert "No services started and nothing was stopped" in message
    assert "kill" not in message


def test_other_socket_errors_are_not_misreported_as_occupied(monkeypatch, capsys):
    def denied(port):
        raise PortPreflightError(port, PermissionError(errno.EACCES, "Permission denied"))
    monkeypatch.setattr("transition_autonomy.startup_ports.check_port_available", denied)
    assert main(["logical"]) == 1
    message = capsys.readouterr().err
    assert "Permission denied" in message
    assert "occupied" not in message
    assert "ss -ltnp" not in message


@pytest.mark.parametrize("mode,ports", [
    ("logical", [10000, 8766]), ("isaac", [10000, 8766, 8767]),
    ("hardware-observation", [10000]),
])
def test_mode_checks_only_its_service_ports(monkeypatch, mode, ports):
    checked = []
    monkeypatch.setattr("transition_autonomy.startup_ports.check_port_available", checked.append)
    monkeypatch.setattr("transition_autonomy.startup_ports.check_stream_port_available",
                        lambda *_: pytest.fail("ordinary startup must not probe stream ports"))
    assert main([mode]) == 0
    assert checked == ports


@pytest.mark.skipif(sys.platform != "linux", reason="Ubuntu stream sockets use Linux bind semantics")
@pytest.mark.parametrize("transport", ["TCP", "UDP"])
@pytest.mark.parametrize("family,address", [(socket.AF_INET, "127.0.0.1"), (socket.AF_INET6, "::1")])
@pytest.mark.parametrize("reuse_address", [False, True])
def test_live_stream_endpoint_is_refused_without_disturbing_owner(transport, family, address, reuse_address):
    kind = socket.SOCK_STREAM if transport == "TCP" else socket.SOCK_DGRAM
    try:
        owner = socket.socket(family, kind)
    except OSError as error:
        if family == socket.AF_INET6 and error.errno in (errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT):
            pytest.skip("IPv6 unavailable")
        raise
    with owner:
        owner.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, int(reuse_address))
        if family == socket.AF_INET6:
            owner.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
        try:
            owner.bind((address, 0))
        except OSError as error:
            if family == socket.AF_INET6 and error.errno == errno.EADDRNOTAVAIL:
                pytest.skip("IPv6 loopback unavailable")
            raise
        port = owner.getsockname()[1]
        owner.settimeout(1)
        if transport == "TCP":
            owner.listen(1)
        with pytest.raises(PortPreflightError) as caught:
            check_stream_port_available(transport, port)
        assert caught.value.error.errno == errno.EADDRINUSE
        assert caught.value.transport == transport
        assert caught.value.address == ("::" if family == socket.AF_INET6 else "0.0.0.0")
        with socket.socket(family, kind) as client:
            client.settimeout(1)
            if transport == "TCP":
                client.connect((address, port))
                connection, _ = owner.accept()
                with connection:
                    connection.sendall(b"original TCP owner")
                    assert client.recv(100) == b"original TCP owner"
            else:
                client.sendto(b"original UDP owner", (address, port))
                assert owner.recvfrom(100)[0] == b"original UDP owner"


@pytest.mark.parametrize("transport", ["TCP", "UDP"])
def test_stream_probe_closes_sockets_on_success(monkeypatch, transport):
    original_socket = socket.socket
    probes = []
    def tracked_socket(*args, **kwargs):
        probe = original_socket(*args, **kwargs)
        probes.append(probe)
        return probe
    monkeypatch.setattr(socket, "socket", tracked_socket)
    check_stream_port_available(transport, 0)
    assert probes
    assert all(probe.fileno() == -1 for probe in probes)


@pytest.mark.parametrize("error_number", [errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL])
def test_stream_ipv6_is_optional(monkeypatch, error_number):
    original_socket = socket.socket
    def without_ipv6(family, kind):
        if family == socket.AF_INET6:
            raise OSError(error_number, "IPv6 unavailable")
        return original_socket(family, kind)
    monkeypatch.setattr(socket, "socket", without_ipv6)
    check_stream_port_available("UDP", 0)


@pytest.mark.parametrize("error_number", [errno.EADDRINUSE, errno.EACCES])
def test_ipv6_conflicts_and_permission_errors_are_not_ignored(monkeypatch, error_number):
    original_socket = socket.socket
    def denied_ipv6(family, kind):
        if family == socket.AF_INET6:
            raise OSError(error_number, "IPv6 endpoint denied")
        return original_socket(family, kind)
    monkeypatch.setattr(socket, "socket", denied_ipv6)
    with pytest.raises(PortPreflightError) as caught:
        check_stream_port_available("UDP", 0)
    assert caught.value.error.errno == error_number
    assert caught.value.address == "::"


def test_udp_failure_identifies_transport_address_and_inspection_command():
    message = failure_message(PortPreflightError(47998, OSError(errno.EADDRINUSE, "Address already in use"),
                                               transport="UDP", address="::"))
    assert "workstation WebRTC [::]:47998 (UDP)" in message
    assert "ss -aunp 'sport = :47998'" in message
    assert "No services started and nothing was stopped" in message


def test_isaac_webrtc_checks_service_and_stream_ports(monkeypatch):
    checked = []
    monkeypatch.setattr("transition_autonomy.startup_ports.check_port_available",
                        lambda port: checked.append(("service", port)))
    monkeypatch.setattr("transition_autonomy.startup_ports.check_stream_port_available",
                        lambda transport, port: checked.append((transport, port)))
    assert main(["isaac", "--webrtc"]) == 0
    assert checked == [("service", 10000), ("service", 8766), ("service", 8767), ("TCP", 49100), ("UDP", 47998)]


@pytest.mark.parametrize("mode", ["logical", "hardware-observation"])
def test_webrtc_rejects_other_modes_before_any_probe(monkeypatch, capsys, mode):
    monkeypatch.setattr("transition_autonomy.startup_ports.check_port_available",
                        lambda *_: pytest.fail("invalid request must not probe sockets"))
    with pytest.raises(SystemExit) as caught:
        main([mode, "--webrtc"])
    assert caught.value.code == 2
    assert "supported only in isaac mode" in capsys.readouterr().err


def test_main_reports_stream_conflict_and_stops_probing(monkeypatch, capsys):
    checked = []
    monkeypatch.setattr("transition_autonomy.startup_ports.check_port_available", lambda _: None)
    def denied(transport, port):
        checked.append((transport, port))
        raise PortPreflightError(port, OSError(errno.EADDRINUSE, "already bound"),
                                 transport=transport, address="0.0.0.0")
    monkeypatch.setattr("transition_autonomy.startup_ports.check_stream_port_available", denied)
    assert main(["isaac", "--webrtc"]) == 1
    assert checked == [("TCP", 49100)]
    assert "WebRTC 0.0.0.0:49100 (TCP)" in capsys.readouterr().err
