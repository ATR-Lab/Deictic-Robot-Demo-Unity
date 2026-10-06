"""Optional Isaac Sim 5.0 spectator streaming; no robot control interfaces.

Uses the settings and extension used by sim/k1_isaac.py and NVIDIA's 5.0
livestream instructions, rather than the different Isaac 6 primaryStream keys:
https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/manual_livestream_clients.html
"""
from __future__ import annotations

from ipaddress import IPv4Address


SPECTATOR_CAMERA = "/World/TaskSpectatorCamera"
STREAMING_EXTENSION = "omni.services.livestream.nvcf"
SIGNAL_PORT = 49100
MEDIA_PORT = 47998  # Isaac Sim 5.0 streaming extension default (UDP).


def public_ipv4(value: str) -> str:
    """Validate an explicit client-reachable address before Isaac is imported."""
    try:
        address = IPv4Address(value)
    except (ValueError, TypeError) as exc:
        raise ValueError("public-ip must be a literal IPv4 address") from exc
    if address.is_unspecified or address.is_multicast or value == "255.255.255.255":
        raise ValueError("public-ip must be a unicast IPv4 address reachable by the client")
    return str(address)


def configure_streaming(app, *, enabled: bool, public_ip: str, enable_extension) -> dict:
    """Set endpoint settings before the extension starts its network services."""
    if not enabled:
        return {"enabled": False, "viewport_camera": None}
    public_ip = public_ipv4(public_ip)
    settings = {
        "/app/window/drawMouse": True,
        "/app/livestream/allowDynamicResize": True,
        "/app/livestream/publicEndpointAddress": public_ip,
        "/app/livestream/port": SIGNAL_PORT,
    }
    for name, value in settings.items():
        app.set_setting(name, value)
    enable_extension(STREAMING_EXTENSION)
    app.update()
    return {
        "enabled": True,
        "extension": STREAMING_EXTENSION,
        "public_ip": public_ip,
        "signaling": {"protocol": "tcp", "port": SIGNAL_PORT},
        "media": {"protocol": "udp", "port": MEDIA_PORT, "source": "Isaac Sim 5.0 extension default"},
        "settings": settings,
        "viewport_camera": None,
        "configuration_source": "Isaac Sim 5.0 livestream documentation and sim/k1_isaac.py",
        "client_connection_verified": False,
    }


def select_spectator_viewport(viewport, provenance: dict) -> None:
    """Select the initialized scene camera, failing instead of streaming an empty view."""
    if viewport is None:
        raise RuntimeError("WebRTC is enabled but Isaac has no active viewport")
    viewport.camera_path = SPECTATOR_CAMERA
    if str(viewport.camera_path) != SPECTATOR_CAMERA:
        raise RuntimeError("WebRTC viewport did not select the task spectator camera")
    provenance["viewport_camera"] = SPECTATOR_CAMERA
