import threading

from diagnostic_msgs.msg import DiagnosticStatus

from map_tools.web_zone_server import ROSBAG_TOPIC_PROFILES, WebZoneServerNode


def _diag_level(value) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    return int(value)


class _FakeNode:
    _diag_level_value = staticmethod(WebZoneServerNode._diag_level_value)
    _should_surface_diagnostic = WebZoneServerNode._should_surface_diagnostic
    _rosbag_topics_for_profile = staticmethod(WebZoneServerNode._rosbag_topics_for_profile)


class _FakeStatus:
    def __init__(self, name: str, level, message: str) -> None:
        self.name = name
        self.level = level
        self.message = message


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeManualNode:
    set_manual_cmd = WebZoneServerNode.set_manual_cmd

    def __init__(self, manual_enabled: bool) -> None:
        self._lock = threading.Lock()
        self._manual_control = {
            "enabled": bool(manual_enabled),
            "linear_x_cmd": 0.0,
            "angular_z_cmd": 0.0,
            "last_cmd_age_s": None,
        }
        self._manual_cmd_last_monotonic = None
        self._teleop_cmd_pub = _FakePublisher()
        self.mode_calls = 0

    def set_manual_mode(self, enabled: bool):
        self.mode_calls += 1
        return True, "", bool(enabled)


def test_should_surface_diagnostic_accepts_navigation_errors():
    node = _FakeNode()
    status = _FakeStatus(
        "navigation/nav_command_server",
        DiagnosticStatus.ERROR,
        "failure=GOAL_RESULT_ABORTED",
    )

    assert node._should_surface_diagnostic(status) is True


def test_should_surface_diagnostic_filters_non_navigation_status():
    node = _FakeNode()
    status = _FakeStatus("ekf_filter_node_map", DiagnosticStatus.ERROR, "stale")

    assert node._should_surface_diagnostic(status) is False


def test_should_surface_diagnostic_filters_idle_collision_monitor_warning():
    node = _FakeNode()
    status = _FakeStatus(
        "navigation/collision_monitor",
        DiagnosticStatus.WARN,
        "no collision monitor state yet",
    )

    assert node._should_surface_diagnostic(status) is False


def test_rosbag_topics_for_profile_matches_declared_profiles():
    topics = _FakeNode._rosbag_topics_for_profile("core")

    assert topics == ROSBAG_TOPIC_PROFILES["core"]
    assert "/diagnostics" in topics
    assert "/nav_command_server/events" in topics
    assert _FakeNode._rosbag_topics_for_profile("missing") is None


def test_set_manual_cmd_publishes_when_manual_disabled() -> None:
    node = _FakeManualNode(manual_enabled=False)

    ok, err = node.set_manual_cmd(1.2, 0.3, 5)

    assert ok is True
    assert err == ""
    assert node.mode_calls == 0
    assert len(node._teleop_cmd_pub.messages) == 1
    published = node._teleop_cmd_pub.messages[0]
    assert published.twist.linear.x == 1.2
    assert published.twist.angular.z == 0.3
    assert published.brake_pct == 5


def test_set_manual_cmd_publishes_when_manual_enabled() -> None:
    node = _FakeManualNode(manual_enabled=True)

    ok, err = node.set_manual_cmd(0.5, -0.2, 0)

    assert ok is True
    assert err == ""
    assert node.mode_calls == 0
    assert len(node._teleop_cmd_pub.messages) == 1


def test_set_manual_cmd_invalid_values_still_fail() -> None:
    node = _FakeManualNode(manual_enabled=False)

    ok, err = node.set_manual_cmd(float("nan"), 0.0, 0)

    assert ok is False
    assert err == "invalid manual command values"
    assert node.mode_calls == 0
    assert len(node._teleop_cmd_pub.messages) == 0


class _FakeSetDatumRequest:
    def __init__(self) -> None:
        self.coords = None


class _FakeSetDatum:
    Request = _FakeSetDatumRequest


class _FakeDatumClient:
    def __init__(self) -> None:
        self.srv_name = "/datum_setter/set_datum"


class _FakeSetDatumNode:
    set_datum_current = WebZoneServerNode.set_datum_current

    def __init__(self, response) -> None:
        self._nav_set_datum_client = _FakeDatumClient()
        self.request_timeout_s = 5.0
        self._response = response
        self.last_request = None

    def _call_service(self, _client, request, _timeout_s):
        self.last_request = request
        return self._response


def test_set_datum_current_sends_empty_coords_and_propagates_success() -> None:
    original = WebZoneServerNode.set_datum_current.__globals__["SetDatum"]
    WebZoneServerNode.set_datum_current.__globals__["SetDatum"] = _FakeSetDatum
    try:
        response = type(
            "Res",
            (),
            {"ok": True, "error": "", "status_message": "Datum set successfully."},
        )()
        node = _FakeSetDatumNode(response=response)
        ok, err = node.set_datum_current()
        assert ok is True
        assert "Datum set successfully." in err
        assert isinstance(node.last_request, _FakeSetDatumRequest)
        assert node.last_request.coords == []
    finally:
        WebZoneServerNode.set_datum_current.__globals__["SetDatum"] = original


def test_set_datum_current_timeout_returns_error() -> None:
    original = WebZoneServerNode.set_datum_current.__globals__["SetDatum"]
    WebZoneServerNode.set_datum_current.__globals__["SetDatum"] = _FakeSetDatum
    try:
        node = _FakeSetDatumNode(response=None)
        ok, err = node.set_datum_current()
        assert ok is False
        assert err == "set_datum timeout"
    finally:
        WebZoneServerNode.set_datum_current.__globals__["SetDatum"] = original


def test_set_datum_current_error_propagates_backend_error() -> None:
    original = WebZoneServerNode.set_datum_current.__globals__["SetDatum"]
    WebZoneServerNode.set_datum_current.__globals__["SetDatum"] = _FakeSetDatum
    try:
        response = type("Res", (), {"ok": False, "error": "no GPS sample", "status_message": ""})()
        node = _FakeSetDatumNode(response=response)
        ok, err = node.set_datum_current()
        assert ok is False
        assert err == "no GPS sample"
    finally:
        WebZoneServerNode.set_datum_current.__globals__["SetDatum"] = original
