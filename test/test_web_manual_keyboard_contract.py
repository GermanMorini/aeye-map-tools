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
