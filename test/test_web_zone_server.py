import asyncio
import json
import threading

from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import Odometry

from map_tools.web_zone_server import (
    ROSBAG_TOPIC_PROFILES,
    SensorInfoSession,
    WebSocketApi,
    WebZoneServerNode,
    _estimated_precision_for_fix,
    _fix_quality_class,
)


def _diag_level(value) -> int:
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    return int(value)


class _FakeNode:
    _diag_level_value = staticmethod(WebZoneServerNode._diag_level_value)
    _should_surface_diagnostic = WebZoneServerNode._should_surface_diagnostic
    _rosbag_topics_for_profile = staticmethod(WebZoneServerNode._rosbag_topics_for_profile)
    _yaw_deg_from_quaternion = WebZoneServerNode._yaw_deg_from_quaternion
    _on_robot_heading_odom = WebZoneServerNode._on_robot_heading_odom


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


class _FakeLogger:
    def __init__(self) -> None:
        self.info_msgs = []
        self.warn_msgs = []
        self.error_msgs = []

    def info(self, msg: str) -> None:
        self.info_msgs.append(str(msg))

    def warning(self, msg: str) -> None:
        self.warn_msgs.append(str(msg))

    def error(self, msg: str) -> None:
        self.error_msgs.append(str(msg))


class _FakeManualNode:
    set_manual_cmd = WebZoneServerNode.set_manual_cmd

    def __init__(self, manual_enabled: bool, control_locked: bool = False) -> None:
        self._lock = threading.Lock()
        self._control_locked = bool(control_locked)
        self._control_lock_reason = "STARTUP_LOCKED" if control_locked else ""
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


class _FakeClosableSession:
    def __init__(self) -> None:
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeRemoveClientNode:
    remove_client = WebZoneServerNode.remove_client
    _engage_lock_if_no_clients = WebZoneServerNode._engage_lock_if_no_clients

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ws_clients = {"ws-1"}
        self._ws_send_locks = {"ws-1": object()}
        self._sensor_info_sessions = {"ws-1": _FakeClosableSession()}
        self.lock_requests = []
        self.logger = _FakeLogger()

    def set_control_lock(self, locked: bool):
        self.lock_requests.append(bool(locked))
        return True, "", bool(locked)

    def get_logger(self):
        return self.logger


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


def test_set_manual_cmd_rejects_when_control_locked() -> None:
    node = _FakeManualNode(manual_enabled=True, control_locked=True)

    ok, err = node.set_manual_cmd(0.5, -0.2, 0)

    assert ok is False
    assert err == "control locked: STARTUP_LOCKED"
    assert node.mode_calls == 0
    assert len(node._teleop_cmd_pub.messages) == 0


def test_remove_client_locks_when_last_ws_client_disconnects() -> None:
    node = _FakeRemoveClientNode()

    node.remove_client("ws-1")

    assert node.lock_requests == [True]
    assert node._ws_clients == set()
    assert node._sensor_info_sessions == {}


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


def test_sensor_info_fix_quality_and_precision_mapping_matches_contract() -> None:
    assert _fix_quality_class("RTK_FIXED") == "good"
    assert _fix_quality_class("RTK_FLOAT") == "warn"
    assert _fix_quality_class("DGPS") == "warn"
    assert _fix_quality_class("3D_FIX") == "bad"
    assert _estimated_precision_for_fix(6) == 0.02
    assert _estimated_precision_for_fix(5) == 0.3
    assert _estimated_precision_for_fix(4) == 0.5
    assert _estimated_precision_for_fix(3) == 3.0
    assert _estimated_precision_for_fix(2) is None


def test_robot_heading_updates_robot_pose_heading_from_odometry_global() -> None:
    node = _FakeNode()
    node._lock = threading.Lock()
    node._last_robot_heading_deg = None
    node._last_robot_pose = {"lat": -31.0, "lon": -64.0}

    msg = Odometry()
    msg.pose.pose.orientation.z = 0.70710678118
    msg.pose.pose.orientation.w = 0.70710678118

    node._on_robot_heading_odom(msg)

    assert node._last_robot_heading_deg is not None
    assert abs(node._last_robot_heading_deg - 90.0) < 1.0e-3
    assert abs(node._last_robot_pose["heading_deg"] - 90.0) < 1.0e-3


class _FakeSensorInfoNode:
    def __init__(self, loop) -> None:
        self._loop = loop
        self.info_gps_topic = "/gps/fix"
        self.info_fix_type_topic = "/gps/fix_type"
        self.info_rtk_status_topic = "/gps/rtk_status"
        self.info_rtcm_age_topic = "/gps/rtcm_age_s"
        self.info_rtcm_count_topic = "/gps/rtcm_received_count"
        self.info_rtk_source_status_topic = "/gps/rtk_source/status_json"
        self.info_gps_raw_topic = "/mavros_node/gps1/raw"
        self.info_imu_topic = "/imu/data"
        self.info_velocity_topic = "/velocity"
        self.info_odom_topic = "/odometry/filtered"
        self.created = []
        self.destroyed = []
        self.sent_payloads = []
        self._topics_catalog = [
            {
                "name": "/parameter_events",
                "types": ["rcl_interfaces/msg/ParameterEvent"],
                "publisher_count": 1,
                "subscriber_count": 0,
            },
            {
                "name": "/topic_alpha",
                "types": ["std_msgs/msg/String"],
                "publisher_count": 2,
                "subscriber_count": 1,
            },
            {
                "name": "/topic_beta",
                "types": ["std_msgs/msg/String"],
                "publisher_count": 1,
                "subscriber_count": 1,
            },
        ]

    def create_subscription(self, msg_type, topic, callback, qos, raw=False):
        token = (msg_type, topic, callback, qos, raw)
        self.created.append(token)
        return token

    def destroy_subscription(self, token):
        self.destroyed.append(token)

    async def send_ws_json(self, _ws, payload):
        self.sent_payloads.append(payload)
        return True

    def get_datum_info(self):
        return {
            "ok": True,
            "already_set": True,
            "has_current_gps": True,
            "gps_is_rtk": True,
            "current_gps_lat": -31.0,
            "current_gps_lon": -64.0,
            "datum_lat": -31.0,
            "datum_lon": -64.0,
            "last_set_epoch_ms": 0,
            "last_set_source": "service_current_gps",
            "last_set_with_rtk": True,
        }

    def get_topics_catalog(self):
        return [dict(item) for item in self._topics_catalog]


async def _configure_sensor_info_session():
    loop = asyncio.get_running_loop()
    node = _FakeSensorInfoNode(loop)
    session = SensorInfoSession(node, ws=object())

    payload_general = session.configure(enabled=True, tab="general", interval_s=0.1)
    await asyncio.sleep(0)
    subs_after_general = session.subscription_count()

    payload_lidar = session.configure(enabled=True, tab="lidar", interval_s=0.1)
    await asyncio.sleep(0)
    subs_after_lidar = session.subscription_count()

    session.close()
    await asyncio.sleep(0)
    return node, payload_general, payload_lidar, subs_after_general, subs_after_lidar


async def _configure_topics_sensor_info_session():
    loop = asyncio.get_running_loop()
    node = _FakeSensorInfoNode(loop)
    session = SensorInfoSession(node, ws=object())

    payload_topics = session.configure(enabled=True, tab="topics", interval_s=0.2)
    await asyncio.sleep(0)
    subs_without_topic = session.subscription_count()

    payload_topic_alpha = session.configure(
        enabled=True,
        tab="topics",
        interval_s=0.2,
        topic_name="/topic_alpha",
    )
    await asyncio.sleep(0)
    subs_with_topic = session.subscription_count()

    payload_topic_beta = session.configure(
        enabled=True,
        tab="topics",
        interval_s=0.2,
        topic_name="/topic_beta",
    )
    await asyncio.sleep(0)
    subs_after_switch = session.subscription_count()

    session.close()
    await asyncio.sleep(0)
    return (
        node,
        payload_topics,
        payload_topic_alpha,
        payload_topic_beta,
        subs_without_topic,
        subs_with_topic,
        subs_after_switch,
    )


def test_sensor_info_session_switches_tabs_and_drops_dynamic_subscriptions() -> None:
    node, payload_general, payload_lidar, subs_after_general, subs_after_lidar = asyncio.run(
        _configure_sensor_info_session()
    )

    assert payload_general["implemented"] is True
    assert subs_after_general > 0
    assert payload_lidar["implemented"] is False
    assert subs_after_lidar == 0
    assert len(node.destroyed) >= subs_after_general
    assert any(item.get("op") == "sensor_info" and item.get("tab") == "lidar" for item in node.sent_payloads)


def test_sensor_info_topics_session_creates_single_dynamic_subscription_per_selected_topic() -> None:
    (
        node,
        payload_topics,
        payload_topic_alpha,
        payload_topic_beta,
        subs_without_topic,
        subs_with_topic,
        subs_after_switch,
    ) = asyncio.run(_configure_topics_sensor_info_session())

    assert payload_topics["implemented"] is True
    assert payload_topics["topic_name"] is None
    assert subs_without_topic == 0
    assert payload_topic_alpha["topic_name"] == "/topic_alpha"
    assert payload_topic_alpha["selected_type"] == "std_msgs/msg/String"
    assert subs_with_topic == 1
    assert payload_topic_beta["topic_name"] == "/topic_beta"
    assert subs_after_switch == 1
    assert len(node.destroyed) >= 1
    assert any(item[-1] is True for item in node.created)


class _FakeWsNode:
    _extract_client_req_id = staticmethod(WebSocketApi._extract_client_req_id)
    _build_ack_payload = WebSocketApi._build_ack_payload

    def __init__(self) -> None:
        self.logger = _FakeLogger()
        self.sent_payloads = []
        self.get_processes_response = (
            True,
            "",
            [
                {
                    "label": "healthcheck",
                    "command": "./tools/healthcheck-lidar.sh",
                    "cwd": "/ros2_ws",
                    "running": False,
                }
            ],
        )
        self.reload_processes_response = (True, "")
        self.start_process_response = (True, "")
        self.stop_process_response = (True, "")
        self.start_process_calls = []
        self.stop_process_calls = []

    def get_logger(self):
        return self.logger

    async def send_ws_json(self, ws, payload):
        self.sent_payloads.append((ws, dict(payload)))
        return True

    def get_processes_state(self):
        return self.get_processes_response

    def reload_processes_catalog(self):
        return self.reload_processes_response

    def start_process_ws(self, ws, process_label: str, output: bool, client_req_id):
        self.start_process_calls.append((ws, process_label, output, client_req_id))
        return self.start_process_response

    def stop_process_ws(self, ws, process_label: str, client_req_id):
        self.stop_process_calls.append((ws, process_label, client_req_id))
        return self.stop_process_response


class _ImmediateFuture:
    def __init__(self, result) -> None:
        self._result = result

    def done(self) -> bool:
        return True

    def result(self):
        return self._result


class _FakeResultFuture:
    def __init__(self, result) -> None:
        self._result = result
        self._callback = None

    def add_done_callback(self, callback) -> None:
        self._callback = callback

    def result(self):
        return self._result

    def fire(self) -> None:
        assert self._callback is not None
        self._callback(self)


class _FakeGoalHandle:
    def __init__(self, result, cancel_response=None) -> None:
        self.accepted = True
        self.result_future = _FakeResultFuture(result)
        self.cancel_calls = 0
        self.cancel_response = cancel_response

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        response = self.cancel_response
        if response is None:
            response = type("CancelResponse", (), {"goals_canceling": [object()]})()
        return _ImmediateFuture(response)


class _FakeActionClient:
    def __init__(self, goal_handle) -> None:
        self.goal_handle = goal_handle
        self.sent_goals = []
        self.feedback_callback = None

    def wait_for_server(self, timeout_sec=None) -> bool:
        _ = timeout_sec
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.sent_goals.append(goal)
        self.feedback_callback = feedback_callback
        return _ImmediateFuture(self.goal_handle)


class _FakeProcessActionNode:
    start_process_ws = WebZoneServerNode.start_process_ws
    stop_process_ws = WebZoneServerNode.stop_process_ws
    _on_process_feedback = WebZoneServerNode._on_process_feedback
    _on_process_result = WebZoneServerNode._on_process_result
    _register_active_process_request = WebZoneServerNode._register_active_process_request
    _process_stop_unavailable_error = WebZoneServerNode._process_stop_unavailable_error
    _process_result_was_cancelled = staticmethod(WebZoneServerNode._process_result_was_cancelled)

    def __init__(self, action_client) -> None:
        self._lock = threading.Lock()
        self._active_process_requests = {}
        self._process_start_action_client = action_client
        self.request_timeout_s = 5.0
        self._loop = object()
        self.logger = _FakeLogger()
        self.sent_payloads = []
        self.get_processes_response = (True, "", [])

    def get_logger(self):
        return self.logger

    def _wait_for_future(self, future, _timeout_s):
        return future.result()

    def get_processes_state(self):
        return self.get_processes_response

    async def send_ws_json(self, ws, payload):
        self.sent_payloads.append((ws, dict(payload)))
        return True


def test_process_state_item_to_payload_matches_contract() -> None:
    item = type(
        "Item",
        (),
        {
            "process": type(
                "Process",
                (),
                {
                    "label": "healthcheck",
                    "command": "./tools/healthcheck-lidar.sh",
                    "cwd": "/ros2_ws",
                },
            )(),
            "running": True,
        },
    )()

    payload = WebZoneServerNode._process_state_item_to_payload(item)

    assert payload == {
        "label": "healthcheck",
        "command": "./tools/healthcheck-lidar.sh",
        "cwd": "/ros2_ws",
        "running": True,
    }


async def _handle_ws_message(raw: str, node: _FakeWsNode):
    api = WebSocketApi(node)
    ws = object()
    await api._handle_message(ws, raw)
    return ws, node.sent_payloads, node.start_process_calls, node.stop_process_calls


def test_ws_get_processes_returns_state_payload() -> None:
    node = _FakeWsNode()

    ws, sent_payloads, _, _ = asyncio.run(
        _handle_ws_message(
            json.dumps({"op": "get_processes", "client_req_id": "req-1"}),
            node,
        )
    )

    assert sent_payloads == [
        (
            ws,
            {
                "op": "process_executor_state",
                "ok": True,
                "process_list": [
                    {
                        "label": "healthcheck",
                        "command": "./tools/healthcheck-lidar.sh",
                        "cwd": "/ros2_ws",
                        "running": False,
                    }
                ],
                "client_req_id": "req-1",
            },
        )
    ]


def test_ws_reload_processes_returns_ack() -> None:
    node = _FakeWsNode()

    ws, sent_payloads, _, _ = asyncio.run(
        _handle_ws_message(
            json.dumps({"op": "reload_processes", "client_req_id": "req-2"}),
            node,
        )
    )

    assert sent_payloads == [
        (
            ws,
            {
                "op": "ack",
                "ok": True,
                "request": "reload_processes",
                "error": None,
                "client_req_id": "req-2",
            },
        )
    ]


def test_ws_start_process_returns_ack_and_passes_args() -> None:
    node = _FakeWsNode()

    ws, sent_payloads, start_calls, stop_calls = asyncio.run(
        _handle_ws_message(
            json.dumps(
                {
                    "op": "start_process",
                    "client_req_id": "req-3",
                    "process": "healthcheck",
                    "output": True,
                }
            ),
            node,
        )
    )

    assert start_calls == [(ws, "healthcheck", True, "req-3")]
    assert stop_calls == []
    assert sent_payloads == [
        (
            ws,
            {
                "op": "ack",
                "ok": True,
                "request": "start_process",
                "error": None,
                "client_req_id": "req-3",
            },
        )
    ]


def test_ws_start_process_rejects_invalid_output_type() -> None:
    node = _FakeWsNode()

    ws, sent_payloads, start_calls, stop_calls = asyncio.run(
        _handle_ws_message(
            json.dumps(
                {
                    "op": "start_process",
                    "client_req_id": "req-4",
                    "process": "healthcheck",
                    "output": "true",
                }
            ),
            node,
        )
    )

    assert start_calls == []
    assert stop_calls == []
    assert sent_payloads == [
        (
            ws,
            {
                "op": "ack",
                "ok": False,
                "request": "start_process",
                "error": "output must be boolean",
                "client_req_id": "req-4",
            },
        )
    ]


def test_ws_stop_process_returns_ack_and_passes_args() -> None:
    node = _FakeWsNode()

    ws, sent_payloads, start_calls, stop_calls = asyncio.run(
        _handle_ws_message(
            json.dumps(
                {
                    "op": "stop_process",
                    "client_req_id": "req-stop-1",
                    "process": "healthcheck",
                }
            ),
            node,
        )
    )

    assert start_calls == []
    assert stop_calls == [(ws, "healthcheck", "req-stop-1")]
    assert sent_payloads == [
        (
            ws,
            {
                "op": "ack",
                "ok": True,
                "request": "stop_process",
                "error": None,
                "client_req_id": "req-stop-1",
            },
        )
    ]


def test_ws_stop_process_requires_process_field() -> None:
    node = _FakeWsNode()

    ws, sent_payloads, start_calls, stop_calls = asyncio.run(
        _handle_ws_message(
            json.dumps(
                {
                    "op": "stop_process",
                    "client_req_id": "req-stop-2",
                }
            ),
            node,
        )
    )

    assert start_calls == []
    assert stop_calls == []
    assert sent_payloads == [
        (
            ws,
            {
                "op": "ack",
                "ok": False,
                "request": "stop_process",
                "error": "process field is required",
                "client_req_id": "req-stop-2",
            },
        )
    ]


def test_start_process_ws_emits_feedback_and_finished_events(monkeypatch) -> None:
    wrapped_result = type(
        "Wrapped",
        (),
        {"result": type("Result", (), {"ok": True, "error": ""})()},
    )()
    goal_handle = _FakeGoalHandle(wrapped_result)
    action_client = _FakeActionClient(goal_handle)
    node = _FakeProcessActionNode(action_client)
    ws = object()

    def _run_now(coro, _loop):
        _ = _loop
        asyncio.run(coro)
        return _ImmediateFuture(None)

    monkeypatch.setattr(
        WebZoneServerNode._on_process_feedback.__globals__["asyncio"],
        "run_coroutine_threadsafe",
        _run_now,
    )

    ok, err = node.start_process_ws(ws, "healthcheck", True, "req-5")

    assert ok is True
    assert err == ""
    assert len(action_client.sent_goals) == 1
    assert action_client.sent_goals[0].process == "healthcheck"
    assert action_client.sent_goals[0].output is True
    assert action_client.feedback_callback is not None

    action_client.feedback_callback(
        type(
            "FeedbackMessage",
            (),
            {"feedback": type("Feedback", (), {"stream": "stdout", "data": "hola\n"})()},
        )()
    )
    goal_handle.result_future.fire()

    assert node.sent_payloads == [
        (
            ws,
            {
                "op": "process_output",
                "process": "healthcheck",
                "stream": "stdout",
                "data": "hola\n",
                "client_req_id": "req-5",
            },
        ),
        (
            ws,
            {
                "op": "process_finished",
                "process": "healthcheck",
                "ok": True,
                "error": "",
                "client_req_id": "req-5",
            },
        ),
    ]


def test_start_process_ws_disables_feedback_when_output_false() -> None:
    wrapped_result = type(
        "Wrapped",
        (),
        {"result": type("Result", (), {"ok": True, "error": ""})()},
    )()
    goal_handle = _FakeGoalHandle(wrapped_result)
    action_client = _FakeActionClient(goal_handle)
    node = _FakeProcessActionNode(action_client)

    ok, err = node.start_process_ws(object(), "healthcheck", False, "req-6")

    assert ok is True
    assert err == ""
    assert len(action_client.sent_goals) == 1
    assert action_client.sent_goals[0].output is False
    assert action_client.feedback_callback is None


def test_stop_process_ws_rejects_unknown_label() -> None:
    wrapped_result = type(
        "Wrapped",
        (),
        {"result": type("Result", (), {"ok": True, "error": ""})()},
    )()
    goal_handle = _FakeGoalHandle(wrapped_result)
    action_client = _FakeActionClient(goal_handle)
    node = _FakeProcessActionNode(action_client)
    node.get_processes_response = (True, "", [])

    ok, err = node.stop_process_ws(object(), "healthcheck", "req-stop-3")

    assert ok is False
    assert err == "unknown process: healthcheck"
    assert goal_handle.cancel_calls == 0


def test_stop_process_ws_rejects_process_not_running() -> None:
    wrapped_result = type(
        "Wrapped",
        (),
        {"result": type("Result", (), {"ok": True, "error": ""})()},
    )()
    goal_handle = _FakeGoalHandle(wrapped_result)
    action_client = _FakeActionClient(goal_handle)
    node = _FakeProcessActionNode(action_client)
    node.get_processes_response = (
        True,
        "",
        [
            {
                "label": "healthcheck",
                "command": "./tools/healthcheck-lidar.sh",
                "cwd": "/ros2_ws",
                "running": False,
            }
        ],
    )

    ok, err = node.stop_process_ws(object(), "healthcheck", "req-stop-4")

    assert ok is False
    assert err == "process not running: healthcheck"
    assert goal_handle.cancel_calls == 0


def test_stop_process_ws_cancels_goal_and_reroutes_finished_event(monkeypatch) -> None:
    wrapped_result = type(
        "Wrapped",
        (),
        {
            "status": 5,
            "result": type("Result", (), {"ok": False, "error": "cancelled"})(),
        },
    )()
    goal_handle = _FakeGoalHandle(wrapped_result)
    action_client = _FakeActionClient(goal_handle)
    node = _FakeProcessActionNode(action_client)
    starter_ws = object()
    stopper_ws = object()

    def _run_now(coro, _loop):
        _ = _loop
        asyncio.run(coro)
        return _ImmediateFuture(None)

    monkeypatch.setattr(
        WebZoneServerNode._on_process_result.__globals__["asyncio"],
        "run_coroutine_threadsafe",
        _run_now,
    )

    ok, err = node.start_process_ws(starter_ws, "healthcheck", False, "req-start")

    assert ok is True
    assert err == ""

    ok, err = node.stop_process_ws(stopper_ws, "healthcheck", "req-stop-5")

    assert ok is True
    assert err == ""
    assert goal_handle.cancel_calls == 1

    goal_handle.result_future.fire()

    assert node.sent_payloads == [
        (
            stopper_ws,
            {
                "op": "process_finished",
                "process": "healthcheck",
                "ok": True,
                "client_req_id": "req-stop-5",
            },
        )
    ]
    assert node._active_process_requests == {}


def test_stop_process_ws_emits_failure_when_result_is_not_cancelled(monkeypatch) -> None:
    wrapped_result = type(
        "Wrapped",
        (),
        {
            "status": 6,
            "result": type("Result", (), {"ok": False, "error": "process exited with code 7"})(),
        },
    )()
    goal_handle = _FakeGoalHandle(wrapped_result)
    action_client = _FakeActionClient(goal_handle)
    node = _FakeProcessActionNode(action_client)
    starter_ws = object()
    stopper_ws = object()

    def _run_now(coro, _loop):
        _ = _loop
        asyncio.run(coro)
        return _ImmediateFuture(None)

    monkeypatch.setattr(
        WebZoneServerNode._on_process_result.__globals__["asyncio"],
        "run_coroutine_threadsafe",
        _run_now,
    )

    assert node.start_process_ws(starter_ws, "healthcheck", False, "req-start-2") == (True, "")
    assert node.stop_process_ws(stopper_ws, "healthcheck", "req-stop-6") == (True, "")

    goal_handle.result_future.fire()

    assert node.sent_payloads == [
        (
            stopper_ws,
            {
                "op": "process_finished",
                "process": "healthcheck",
                "ok": False,
                "error": "process exited with code 7",
                "client_req_id": "req-stop-6",
            },
        )
    ]
