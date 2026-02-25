#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ProjectionStatus:
    ok: bool
    message: str


class ZoneProjection:
    """Non-blocking client wrapper for robot_localization fromLL/toLL services."""

    def __init__(self, node: Any, from_ll_service: str, to_ll_service: str) -> None:
        self._node = node
        self._from_ll_client: Any | None = None
        self._to_ll_client: Any | None = None
        self._from_ll_type: Any | None = None
        self._to_ll_type: Any | None = None
        self._status = ProjectionStatus(ok=False, message="projection disabled")

        try:
            from robot_localization.srv import FromLL  # type: ignore
            from robot_localization.srv import ToLL  # type: ignore
        except ImportError:
            self._status = ProjectionStatus(
                ok=False,
                message="robot_localization services are not available in this environment",
            )
            return

        self._from_ll_type = FromLL
        self._to_ll_type = ToLL
        self._from_ll_client = node.create_client(FromLL, from_ll_service)
        self._to_ll_client = node.create_client(ToLL, to_ll_service)
        self._status = ProjectionStatus(ok=True, message="projection ready")

    @property
    def status(self) -> ProjectionStatus:
        return self._status

    def services_ready(self) -> tuple[bool, str]:
        if not self._status.ok:
            return (False, self._status.message)
        assert self._from_ll_client is not None
        assert self._to_ll_client is not None
        if not self._from_ll_client.service_is_ready():
            return (False, "fromLL service is not ready")
        if not self._to_ll_client.service_is_ready():
            return (False, "toLL service is not ready")
        return (True, "ok")

    def ll_to_map_async(self, latitude: float, longitude: float, altitude: float = 0.0) -> Any:
        if not self._status.ok:
            return None
        assert self._from_ll_type is not None
        assert self._from_ll_client is not None

        req = self._from_ll_type.Request()
        self._set_ll_request(req, latitude, longitude, altitude)
        return self._from_ll_client.call_async(req)

    def map_to_ll_async(self, x: float, y: float, z: float = 0.0) -> Any:
        if not self._status.ok:
            return None
        assert self._to_ll_type is not None
        assert self._to_ll_client is not None

        req = self._to_ll_type.Request()
        self._set_map_request(req, x, y, z)
        return self._to_ll_client.call_async(req)

    def parse_from_ll_response(self, response: Any) -> dict[str, float]:
        map_point = getattr(response, "map_point", None)
        if map_point is None:
            raise ValueError("fromLL response missing map_point")
        x = float(getattr(map_point, "x"))
        y = float(getattr(map_point, "y"))
        z = float(getattr(map_point, "z", 0.0))
        return {"x": x, "y": y, "z": z}

    def parse_to_ll_response(self, response: Any) -> dict[str, float]:
        ll_point = getattr(response, "ll_point", None)
        if ll_point is None:
            raise ValueError("toLL response missing ll_point")

        if hasattr(ll_point, "latitude"):
            latitude = float(getattr(ll_point, "latitude"))
            longitude = float(getattr(ll_point, "longitude"))
            altitude = float(getattr(ll_point, "altitude", 0.0))
            return {"lat": latitude, "lon": longitude, "alt": altitude}

        latitude = float(getattr(ll_point, "y"))
        longitude = float(getattr(ll_point, "x"))
        altitude = float(getattr(ll_point, "z", 0.0))
        return {"lat": latitude, "lon": longitude, "alt": altitude}

    def _set_ll_request(self, req: Any, latitude: float, longitude: float, altitude: float) -> None:
        ll_point = getattr(req, "ll_point", None)
        if ll_point is None:
            setattr(req, "latitude", float(latitude))
            setattr(req, "longitude", float(longitude))
            if hasattr(req, "altitude"):
                setattr(req, "altitude", float(altitude))
            return

        if hasattr(ll_point, "latitude"):
            ll_point.latitude = float(latitude)
            ll_point.longitude = float(longitude)
            ll_point.altitude = float(altitude)
            return

        ll_point.x = float(longitude)
        ll_point.y = float(latitude)
        ll_point.z = float(altitude)

    def _set_map_request(self, req: Any, x: float, y: float, z: float) -> None:
        map_point = getattr(req, "map_point", None)
        if map_point is None:
            setattr(req, "x", float(x))
            setattr(req, "y", float(y))
            if hasattr(req, "z"):
                setattr(req, "z", float(z))
            return

        map_point.x = float(x)
        map_point.y = float(y)
        map_point.z = float(z)
