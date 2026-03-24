from pathlib import Path


def test_manual_keyboard_does_not_require_manual_preenabled() -> None:
    index_path = Path(__file__).resolve().parents[1] / "web" / "index.html"
    contents = index_path.read_text(encoding="utf-8")

    assert "const isManualKey =" in contents
    assert "manualControlTick();" in contents
    assert "if (!state.manualControl.enabled) return;" not in contents
    assert "set({ op: 'set_manual_mode', enabled: true })" not in contents
