"""Offline checks for opt-in streaming setup; native media is verified separately."""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    "isaac_streaming", Path(__file__).resolve().parents[1] / "deployment/isaac_streaming.py")
streaming = importlib.util.module_from_spec(spec)
spec.loader.exec_module(streaming)


class App:
    def __init__(self):
        self.calls = []

    def set_setting(self, key, value):
        self.calls.append(("setting", key, value))

    def update(self):
        self.calls.append(("update",))


def test_disabled_streaming_never_initializes_extension_or_settings():
    app = App()
    def unexpected_extension(_):
        pytest.fail("disabled worker attempted to enable streaming")
    result = streaming.configure_streaming(
        app, enabled=False, public_ip="", enable_extension=unexpected_extension)
    assert app.calls == []
    assert result == {"enabled": False, "viewport_camera": None}


@pytest.mark.parametrize("value", ["", "mlworkstation.atr.cs.kent.edu", "::1", "192.168.0.1;echo x",
                                  "0.0.0.0", "239.1.2.3", "255.255.255.255", "192.168.001.2"])
def test_invalid_address_fails_before_any_app_mutation(value):
    app = App()
    with pytest.raises(ValueError, match="IPv4"):
        streaming.configure_streaming(app, enabled=True, public_ip=value,
            enable_extension=lambda name: app.calls.append(("extension", name)))
    assert app.calls == []


def test_endpoint_is_configured_before_extension_binds():
    app = App()
    result = streaming.configure_streaming(app, enabled=True, public_ip="192.0.2.12",
        enable_extension=lambda name: app.calls.append(("extension", name)))
    assert app.calls[-2:] == [("extension", streaming.STREAMING_EXTENSION), ("update",)]
    settings = {call[1]: call[2] for call in app.calls[:-2]}
    assert settings["/app/livestream/publicEndpointAddress"] == "192.0.2.12"
    assert settings["/app/livestream/port"] == 49100
    assert result["media"]["port"] == 47998
    assert result["client_connection_verified"] is False
    assert result["viewport_camera"] is None


def test_missing_viewport_fails_without_claiming_selected_camera():
    result = {"viewport_camera": None}
    with pytest.raises(RuntimeError, match="no active viewport"):
        streaming.select_spectator_viewport(None, result)
    assert result["viewport_camera"] is None


def test_viewport_rejecting_camera_is_detected():
    class StaleViewport:
        @property
        def camera_path(self):
            return "/OmniverseKit_Persp"

        @camera_path.setter
        def camera_path(self, value):
            pass
    with pytest.raises(RuntimeError, match="did not select"):
        streaming.select_spectator_viewport(StaleViewport(), {})


def test_initialized_spectator_becomes_the_streamed_view():
    class Viewport:
        camera_path = "/OmniverseKit_Persp"
    viewport, result = Viewport(), {}
    streaming.select_spectator_viewport(viewport, result)
    assert viewport.camera_path == "/World/TaskSpectatorCamera"
    assert result["viewport_camera"] == viewport.camera_path
