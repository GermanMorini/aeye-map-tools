from pathlib import Path


def test_manual_keyboard_does_not_require_manual_preenabled() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "const isManualKey =" in contents
    assert "manualControlTick();" in contents
    assert "if (!state.manualControl.enabled) return;" not in contents
    assert "set({ op: 'set_manual_mode', enabled: true })" not in contents


def test_map_toolbar_exposes_set_datum_button_and_ws_operation() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    center_idx = contents.find('id="mapToolCenterRobotBtn"')
    datum_idx = contents.find('id="mapToolSetDatumBtn"')
    close_idx = contents.find('id="mapToolCloseBtn"')
    assert center_idx != -1 and datum_idx != -1 and close_idx != -1
    assert center_idx < datum_idx < close_idx
    assert "send({ op: 'set_datum' });" in contents
    assert "msg.request === 'set_datum'" in contents


def test_connection_panel_exposes_record_and_info_actions() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert 'id="infoBtn"' in contents
    assert '<span class="btn-label">Record</span>' in contents
    assert "<h3>Record</h3>" in contents
    assert "<h3>Menu</h3>" not in contents


def test_info_modal_contract_is_present() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert 'id="infoModal"' in contents
    assert 'data-info-tab="general"' in contents
    assert 'data-info-tab="topics"' in contents
    assert 'data-info-tab="pixhawk_gps"' in contents
    assert 'data-info-tab="lidar"' in contents
    assert 'data-info-tab="camera"' in contents
    assert 'id="infoRefreshIntervalInput"' in contents
    assert "set_sensor_info_view" in contents
    assert "msg.op === 'sensor_info'" in contents
    assert 'id="infoTopicsSearchInput"' in contents
    assert 'id="infoTopicsCopyBtn"' in contents
    assert "selectSensorInfoTopic" in contents
    assert 'class="info-topics-selected-meta"' in contents


def test_keyboard_shortcut_i_opens_info_modal_when_not_typing() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "const isPlainInfoShortcut = !event.ctrlKey" in contents
    assert "&& (code === 'KeyI' || key === 'i' || key === 'I');" in contents
    assert "if (isTextInputFocused()) return;" in contents
    assert "if (isPlainInfoShortcut) {" in contents
    assert "if (!state.sensorInfo.open) {" in contents
    assert "openInfoModal();" in contents


def test_navigation_lock_ui_and_heartbeat_contract_are_present() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert 'id="navControls"' in contents
    assert 'id="unlockControlOverlay"' in contents
    assert 'id="unlockControlBtn"' in contents
    assert 'id="unlockControlText"' in contents
    assert "← Desbloquear" in contents
    assert "CONTROL_HEARTBEAT_INTERVAL_MS = 2000;" in contents
    assert "op: 'set_control_lock', locked: false" in contents
    assert "op: 'control_heartbeat'" in contents
    assert "function syncControlLockFromServer(locked, reason, options = {})" in contents
    assert "formatControlLockReason(state.controlLock.reason)" in contents
    assert "overlay.style.display = locked ? 'flex' : 'none';" in contents
    assert "navControls.style.pointerEvents = 'auto';" in contents
    assert "document.getElementById('sendGoalsBtn').disabled = locked || !hasWaypoints;" in contents
    assert "document.getElementById('manualModeBtn').disabled = locked;" in contents
    assert "document.getElementById('popGoalBtn').disabled = !hasWaypoints;" in contents
    assert "unlockGraceUntilMs: 0" in contents
    assert "startUnlockGrace: msg.ok && !lockedAfter" in contents
    assert 'id="navLockedView"' not in contents
    assert 'id="navUnlockedControls"' not in contents


def test_manual_disable_pending_contract_is_present() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "manualDisablePending: false," in contents
    assert "if (state.manualDisablePending) return;" in contents
    assert "if (state.manualDisablePending) {" in contents
    assert "const disablePending = state.manualDisablePending === true;" in contents
    assert "btn.textContent = disablePending ? '...' : (enabled ? 'ON' : 'OFF');" in contents
    assert "state.manualDisablePending = true;" in contents
    assert "state.manualDisablePending = false;" in contents
    assert "if (state.manualDisablePending && enabledFromServer) {" in contents


def test_nav_event_can_resync_control_lock_state() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "function syncControlLockFromNavEvent(eventPayload) {" in contents
    assert "const code = String(eventPayload.code || '').trim();" in contents
    assert "if (code === 'CONTROL_LOCK_RELEASED') {" in contents
    assert "syncControlLockFromServer(false, detailReason);" in contents
    assert "if (code === 'CONTROL_LOCK_ENGAGED') {" in contents
    assert "syncControlLockFromServer(true, detailReason || 'UI_LOCK_REQUEST');" in contents
    assert "if (code === 'UI_HEARTBEAT_TIMEOUT') {" in contents
    assert "syncControlLockFromServer(true, detailReason || 'UI_HEARTBEAT_TIMEOUT');" in contents
    assert "pushRecentEvent(eventPayload);" in contents
    assert "syncControlLockFromNavEvent(eventPayload);" in contents


def test_robot_icon_does_not_treat_null_heading_as_east() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "const hasHeading = headingDeg !== null" in contents
    assert "&& headingDeg !== undefined" in contents
    assert "Number.isFinite(Number(headingDeg));" in contents


def test_waypoint_yaw_uses_local_north_east_meters_instead_of_raw_degree_deltas() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "const metersPerDegLat = 111320.0;" in contents
    assert "const metersPerDegLon = metersPerDegLat * Math.max(1.0e-6, Math.abs(Math.cos(refLat * Math.PI / 180.0)));" in contents
    assert "const eastM = (Number(target.lng) - Number(origin.lng)) * metersPerDegLon;" in contents
    assert "const northM = (Number(target.lat) - Number(origin.lat)) * metersPerDegLat;" in contents

def test_web_index_supports_connection_overrides_from_query_string() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "function readConnectionQueryOverrides()" in contents
    assert "new URLSearchParams(window.location.search || '')" in contents
    assert "params.get('preset')" in contents
    assert "params.get('host')" in contents
    assert "params.get('port')" in contents
    assert "params.get('autoconnect')" in contents
    assert "autostartConnect: false" in contents
    assert "state.autostartConnect = queryConnection.autoconnect === true;" in contents
    assert "if (state.autostartConnect) {" in contents
    assert "connectWs();" in contents


def test_web_index_constrains_goal_spacing_and_heading() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert 'id="goalLoop" type="checkbox" style="width:auto;"' in contents
    assert 'id="goalMinSpacing"' in contents
    assert 'id="manualLinearSlider" type="range" min="1.2"' in contents
    assert "const goalRequest = (payloadWaypoints.length === 1)" in contents
    assert "lat: Number(payloadWaypoints[0].lat)" in contents
    assert "yaw_deg: Number(payloadWaypoints[0].yaw_deg || 0.0)" in contents
    assert "const DEFAULT_GOAL_MIN_SPACING_M = 6.0;" in contents
    assert "const MAX_GOAL_YAW_DELTA_FROM_ROUTE_DEG = 85.0;" in contents
    assert "const MAX_GOAL_TURNAROUND_DELTA_DEG = 135.0;" in contents
    assert "function clampYawAroundReference" in contents
    assert "function preventFullTurnGoalYaw(rawYawDeg, routeYawDeg)" in contents
    assert "if (yawDeltaDeg >= MAX_GOAL_TURNAROUND_DELTA_DEG) {" in contents
    assert "function buildConstrainedGoalWaypoints(rawWaypoints)" in contents
    assert "state.goalWaypoints = buildConstrainedGoalWaypoints(state.goalWaypoints);" in contents
    assert "document.getElementById('goalMinSpacing').addEventListener('change'" in contents
    assert "sin giros 360 y gap minimo" in contents
