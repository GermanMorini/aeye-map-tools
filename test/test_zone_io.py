import pytest

from map_tools.zone_io import ZoneDocument
from map_tools.zone_io import ZoneValidationError
from map_tools.zone_io import load_zone_document
from map_tools.zone_io import save_zone_document
from map_tools.zone_io import validate_document


def test_validate_document_accepts_valid_zone() -> None:
    raw = {
        "frame_id": "map",
        "zones": [
            {
                "id": "zone_a",
                "type": "no_go",
                "enabled": True,
                "polygon": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            }
        ],
    }

    doc = validate_document(raw)

    assert doc.frame_id == "map"
    assert len(doc.zones) == 1
    assert doc.zones[0].zone_id == "zone_a"


def test_validate_document_rejects_self_intersection() -> None:
    raw = {
        "frame_id": "map",
        "zones": [
            {
                "id": "bad",
                "type": "no_go",
                "enabled": True,
                "polygon": [[0.0, 0.0], [2.0, 2.0], [0.0, 2.0], [2.0, 0.0]],
            }
        ],
    }

    with pytest.raises(ZoneValidationError):
        validate_document(raw)


def test_save_load_round_trip(tmp_path) -> None:
    zone_path = tmp_path / "zones.yaml"
    source = ZoneDocument(
        frame_id="map",
        zones=validate_document(
            {
                "frame_id": "map",
                "zones": [
                    {
                        "id": "zone_a",
                        "type": "no_go",
                        "enabled": True,
                        "polygon": [[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]],
                    }
                ],
            }
        ).zones,
    )

    save_zone_document(str(zone_path), source)
    loaded = load_zone_document(str(zone_path))

    assert loaded.frame_id == "map"
    assert len(loaded.zones) == 1
    assert loaded.zones[0].zone_id == "zone_a"
