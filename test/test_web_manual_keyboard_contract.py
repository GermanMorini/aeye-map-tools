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
