import asyncio
import base64
import json
import threading
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
import websockets
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from interfaces.msg import NavTelemetry
from interfaces.srv import (
    BrakeNav,
    CameraPan,
    CameraStatus,
    CancelNavGoal,
    GetNavSnapshot,
    GetNavState,
    GetZonesState,
    SetManualMode,
    SetNavGoalLL,
    SetZonesGeoJson,
)


class WebZoneServerNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("web_zone_server")
        self._loop = loop

        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8766)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("gps_broadcast_hz", 1.0)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("snapshot_request_timeout_s", 2.0)
        self.declare_parameter("set_zones_timeout_s", 12.0)
        self.declare_parameter("set_goal_timeout_s", 12.0)

        self.declare_parameter("zones_set_geojson_service", "/zones_manager/set_geojson")
        self.declare_parameter("zones_get_state_service", "/zones_manager/get_state")
        self.declare_parameter("zones_reload_service", "/zones_manager/reload_from_disk")

        self.declare_parameter("nav_set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("nav_cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("nav_brake_service", "/nav_command_server/brake")
        self.declare_parameter("nav_set_manual_mode_service", "/nav_command_server/set_manual_mode")
        self.declare_parameter("nav_get_state_service", "/nav_command_server/get_state")
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")

        self.declare_parameter("nav_snapshot_service", "/nav_snapshot_server/get_nav_snapshot")
        self.declare_parameter("nav_telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("camera_pan_service", "/camara/camera_pan")
        self.declare_parameter("camera_zoom_toggle_service", "/camara/camera_zoom_toggle")
        self.declare_parameter("camera_status_service", "/camara/camera_status")

        self.ws_host = str(self.get_parameter("ws_host").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.gps_broadcast_hz = float(self.get_parameter("gps_broadcast_hz").value)
        self.request_timeout_s = max(0.5, float(self.get_parameter("request_timeout_s").value))
        self.snapshot_request_timeout_s = max(
            0.5, float(self.get_parameter("snapshot_request_timeout_s").value)
        )
        self.set_zones_timeout_s = max(
            self.request_timeout_s, float(self.get_parameter("set_zones_timeout_s").value)
        )
        self.set_goal_timeout_s = max(
            self.request_timeout_s, float(self.get_parameter("set_goal_timeout_s").value)
        )

        self.zones_set_geojson_service = str(
            self.get_parameter("zones_set_geojson_service").value
        )
        self.zones_get_state_service = str(
            self.get_parameter("zones_get_state_service").value
        )
        self.zones_reload_service = str(self.get_parameter("zones_reload_service").value)

        self.nav_set_goal_service = str(self.get_parameter("nav_set_goal_service").value)
        self.nav_cancel_goal_service = str(
            self.get_parameter("nav_cancel_goal_service").value
        )
        self.nav_brake_service = str(self.get_parameter("nav_brake_service").value)
        self.nav_set_manual_mode_service = str(
            self.get_parameter("nav_set_manual_mode_service").value
        )
        self.nav_get_state_service = str(self.get_parameter("nav_get_state_service").value)
        self.teleop_cmd_topic = str(self.get_parameter("teleop_cmd_topic").value)

        self.nav_snapshot_service = str(self.get_parameter("nav_snapshot_service").value)
        self.nav_telemetry_topic = str(self.get_parameter("nav_telemetry_topic").value)
        self.camera_pan_service = str(self.get_parameter("camera_pan_service").value)
        self.camera_zoom_toggle_service = str(
            self.get_parameter("camera_zoom_toggle_service").value
        )
        self.camera_status_service = str(self.get_parameter("camera_status_service").value)

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()

        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_gps_broadcast_monotonic: Optional[float] = None

        self._zones: List[Dict[str, Any]] = []
        self._zones_geojson: Dict[str, Any] = {"type": "FeatureCollection", "features": []}
        self._mask_ready = False
        self._mask_source = "none"

        self._cmd_vel_safe = {
            "available": False,
            "linear_x": 0.0,
            "angular_z": 0.0,
        }
        self._manual_control = {
            "enabled": False,
            "linear_x_cmd": 0.0,
            "angular_z_cmd": 0.0,
            "last_cmd_age_s": None,
        }
        self._goal_active = False
        self._camera_status = {
            "ok": False,
            "error": "camera status unavailable",
            "last_command": "none",
            "zoom_in": False,
        }

        self._manual_cmd_last_monotonic: Optional[float] = None

        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, 10
        )
        self._nav_telemetry_sub = self.create_subscription(
            NavTelemetry, self.nav_telemetry_topic, self._on_nav_telemetry, 10
        )

        self._zones_set_geojson_client = self.create_client(
            SetZonesGeoJson, self.zones_set_geojson_service
        )
        self._zones_get_state_client = self.create_client(
            GetZonesState, self.zones_get_state_service
        )
        self._zones_reload_client = self.create_client(Trigger, self.zones_reload_service)
        self._nav_set_goal_client = self.create_client(SetNavGoalLL, self.nav_set_goal_service)
        self._nav_cancel_goal_client = self.create_client(
            CancelNavGoal, self.nav_cancel_goal_service
        )
        self._nav_brake_client = self.create_client(BrakeNav, self.nav_brake_service)
        self._nav_set_manual_mode_client = self.create_client(
            SetManualMode, self.nav_set_manual_mode_service
        )
        self._teleop_cmd_pub = self.create_publisher(Twist, self.teleop_cmd_topic, 10)
        self._nav_get_state_client = self.create_client(GetNavState, self.nav_get_state_service)
        self._nav_snapshot_client = self.create_client(GetNavSnapshot, self.nav_snapshot_service)
        self._camera_pan_client = self.create_client(CameraPan, self.camera_pan_service)
        self._camera_zoom_toggle_client = self.create_client(
            Trigger, self.camera_zoom_toggle_service
        )
        self._camera_status_client = self.create_client(
            CameraStatus, self.camera_status_service
        )
        self.get_logger().info(
            "Web gateway ready "
            f"(ws={self.ws_host}:{self.ws_port}, zones_set={self.zones_set_geojson_service}, "
            f"goal_set={self.nav_set_goal_service}, snapshot={self.nav_snapshot_service}, "
            f"camera_pan={self.camera_pan_service}, camera_zoom_toggle={self.camera_zoom_toggle_service}, "
            f"camera_status={self.camera_status_service}, "
            f"teleop_topic={self.teleop_cmd_topic})"
        )

    def add_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.add(ws)
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client connected (clients={count})")

    def remove_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.discard(ws)
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client disconnected (clients={count})")

    def snapshot_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "op": "state",
                "ok": True,
                "frame_id": self.map_frame,
                "zones": list(self._zones),
                "geojson": dict(self._zones_geojson),
                "mask_ready": bool(self._mask_ready),
                "mask_source": str(self._mask_source),
                "robot_pose": self._last_robot_pose,
                "cmd_vel_safe": dict(self._cmd_vel_safe),
                "manual_control": dict(self._manual_control),
                "goal_active": bool(self._goal_active),
                "camera_status": dict(self._camera_status),
            }

    def _build_nav_telemetry_payload(self) -> Dict[str, Any]:
        with self._lock:
            cmd_vel_safe = dict(self._cmd_vel_safe)
            manual_control = dict(self._manual_control)
        return {
            "op": "nav_telemetry",
            "cmd_vel_safe": cmd_vel_safe,
            "manual_control": manual_control,
        }

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
        text = json.dumps(payload)
        with self._lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        failed = []
        for ws in clients:
            try:
                await ws.send(text)
            except Exception:
                failed.append(ws)
        if failed:
            with self._lock:
                for ws in failed:
                    self._ws_clients.discard(ws)

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not np.isfinite(msg.latitude) or not np.isfinite(msg.longitude):
            return

        pose = {"lat": float(msg.latitude), "lon": float(msg.longitude)}
        with self._lock:
            self._last_robot_pose = pose
            last_sent = self._last_gps_broadcast_monotonic

        min_interval = 1.0 / max(0.1, float(self.gps_broadcast_hz))
        now = time.monotonic()
        if last_sent is not None and (now - last_sent) < min_interval:
            return

        with self._lock:
            self._last_gps_broadcast_monotonic = now

        payload = {"op": "robot_pose", "pose": pose}
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        with self._lock:
            self._cmd_vel_safe = {
                "available": bool(msg.cmd_vel_available),
                "linear_x": float(msg.cmd_vel_linear_x),
                "angular_z": float(msg.cmd_vel_angular_z),
            }
            self._goal_active = bool(msg.goal_active)

            last_cmd_age = None
            if self._manual_cmd_last_monotonic is not None:
                last_cmd_age = max(0.0, time.monotonic() - self._manual_cmd_last_monotonic)

            self._manual_control = {
                "enabled": bool(msg.manual_enabled),
                "linear_x_cmd": float(msg.manual_linear_x_cmd),
                "angular_z_cmd": float(msg.manual_angular_z_cmd),
                "last_cmd_age_s": last_cmd_age,
            }

            if np.isfinite(msg.robot_lat) and np.isfinite(msg.robot_lon):
                self._last_robot_pose = {
                    "lat": float(msg.robot_lat),
                    "lon": float(msg.robot_lon),
                }

        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )

    def _wait_for_future(self, future: Any, timeout_s: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_s:
                return None
            time.sleep(0.01)
        return None

    def _call_service(self, client: Any, request: Any, timeout_s: float) -> Optional[Any]:
        service_name = getattr(client, "srv_name", "<unknown_service>")
        request_name = type(request).__name__
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            self.get_logger().warning(
                f"Service unavailable: {service_name} (request={request_name})"
            )
            return None
        future = client.call_async(request)
        result = self._wait_for_future(future, timeout_s)
        if result is None:
            self.get_logger().warning(
                f"Service timeout: {service_name} (request={request_name}, timeout_s={timeout_s:.2f})"
            )
        return result

    def _geojson_string_to_zones(self, geojson_text: str) -> List[Dict[str, Any]]:
        try:
            payload = json.loads(geojson_text)
        except Exception:
            return []
        if not isinstance(payload, dict):
            return []
        if str(payload.get("type", "")) != "FeatureCollection":
            return []
        features = payload.get("features")
        if not isinstance(features, list):
            return []

        zones: List[Dict[str, Any]] = []
        for feature_idx, feature in enumerate(features):
            if not isinstance(feature, dict):
                continue
            props = feature.get("properties")
            if not isinstance(props, dict):
                props = {}

            zone_id = str(props.get("id", f"zone_{feature_idx + 1}"))
            zone_type = str(props.get("type", "no_go"))
            enabled = bool(props.get("enabled", True))

            geometry = feature.get("geometry")
            if not isinstance(geometry, dict):
                continue
            geometry_type = str(geometry.get("type", ""))
            coordinates = geometry.get("coordinates", [])
            polygons: List[Any] = []
            if geometry_type == "Polygon":
                polygons = [coordinates]
            elif geometry_type == "MultiPolygon" and isinstance(coordinates, list):
                polygons = coordinates
            else:
                continue

            for poly_idx, polygon in enumerate(polygons):
                if not isinstance(polygon, list) or len(polygon) == 0:
                    continue
                outer = polygon[0]
                if not isinstance(outer, list):
                    continue
                points: List[Dict[str, float]] = []
                for coord in outer:
                    if not isinstance(coord, (list, tuple)) or len(coord) < 2:
                        continue
                    try:
                        lon = float(coord[0])
                        lat = float(coord[1])
                    except Exception:
                        continue
                    points.append({"lat": lat, "lon": lon})
                if len(points) < 3:
                    continue
                if points[0] == points[-1]:
                    points = points[:-1]
                if len(points) < 3:
                    continue

                polygon_id = zone_id if len(polygons) == 1 else f"{zone_id}__{poly_idx + 1}"
                zones.append(
                    {
                        "id": polygon_id,
                        "type": zone_type,
                        "enabled": enabled,
                        "polygon": points,
                    }
                )
        return zones

    def _normalize_geojson_payload(
        self, payload: Any
    ) -> Tuple[Optional[str], Optional[Dict[str, Any]], str]:
        try:
            if isinstance(payload, str):
                obj = json.loads(payload)
            elif isinstance(payload, dict):
                obj = payload
            else:
                return None, None, "geojson must be an object or string"
        except Exception as exc:
            return None, None, f"invalid geojson json: {exc}"
        if not isinstance(obj, dict):
            return None, None, "geojson root must be an object"
        if str(obj.get("type", "")) != "FeatureCollection":
            return None, None, "geojson root type must be FeatureCollection"
        features = obj.get("features")
        if not isinstance(features, list):
            return None, None, "geojson.features must be a list"
        return json.dumps(obj, ensure_ascii=True), obj, ""

    def _update_zones_state(self, response: GetZonesState.Response) -> None:
        geojson_text = str(response.geojson)
        zones_geojson: Dict[str, Any]
        try:
            parsed = json.loads(geojson_text) if geojson_text else {}
            zones_geojson = parsed if isinstance(parsed, dict) else {}
        except Exception:
            zones_geojson = {}

        with self._lock:
            self._zones = self._geojson_string_to_zones(geojson_text)
            self._zones_geojson = (
                zones_geojson
                if zones_geojson
                else {"type": "FeatureCollection", "features": []}
            )
            self._mask_ready = bool(response.mask_ready)
            self._mask_source = str(response.mask_source)
            if response.frame_id:
                self.map_frame = str(response.frame_id)

    def _update_nav_state(self, response: GetNavState.Response) -> None:
        with self._lock:
            self._goal_active = bool(response.goal_active)
            self._cmd_vel_safe = {
                "available": bool(response.cmd_vel_available),
                "linear_x": float(response.cmd_vel_linear_x),
                "angular_z": float(response.cmd_vel_angular_z),
            }
            self._manual_control = {
                "enabled": bool(response.manual_enabled),
                "linear_x_cmd": float(response.manual_linear_x_cmd),
                "angular_z_cmd": float(response.manual_angular_z_cmd),
                "last_cmd_age_s": None,
            }
            if np.isfinite(response.robot_lat) and np.isfinite(response.robot_lon):
                self._last_robot_pose = {
                    "lat": float(response.robot_lat),
                    "lon": float(response.robot_lon),
                }

    def get_zones_state(self) -> Tuple[bool, str]:
        req = GetZonesState.Request()
        res = self._call_service(self._zones_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "zones get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_zones_state(res)
        return True, ""

    def set_zones_geojson(self, payload: Any) -> Tuple[bool, str, bool]:
        geojson_text, _, err = self._normalize_geojson_payload(payload)
        if geojson_text is None:
            return False, err, False
        self.get_logger().info("WS->ROS set_zones_geojson")
        req = SetZonesGeoJson.Request()
        req.geojson = geojson_text
        res = self._call_service(self._zones_set_geojson_client, req, self.set_zones_timeout_s)
        if res is None:
            return False, "zones set_geojson timeout", False
        if not res.ok:
            self.get_logger().warning(f"set_zones_geojson failed: {res.error}")
            return False, str(res.error), bool(res.map_reloaded)
        self.get_zones_state()
        self.get_logger().info(
            "set_zones_geojson ok "
            f"(features={int(res.feature_count)}, polygons={int(res.polygon_count)}, "
            f"map_reloaded={bool(res.map_reloaded)})"
        )
        return True, "", bool(res.map_reloaded)

    def reload_zones_from_disk(self) -> Tuple[bool, str]:
        self.get_logger().info("WS->ROS reload_zones_from_disk")
        req = Trigger.Request()
        res = self._call_service(
            self._zones_reload_client,
            req,
            self.set_zones_timeout_s,
        )
        if res is None:
            return False, "zones reload timeout"
        if not res.success:
            return False, str(res.message or "reload failed")
        ok_state, err_state = self.get_zones_state()
        if not ok_state:
            return False, err_state
        return True, ""

    def get_nav_state(self) -> Tuple[bool, str]:
        req = GetNavState.Request()
        res = self._call_service(self._nav_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "nav get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_nav_state(res)
        return True, ""

    def set_nav_goals(
        self, waypoints: List[Dict[str, float]], loop: bool
    ) -> Tuple[bool, str, int, bool]:
        if len(waypoints) == 0:
            return False, "at least one waypoint is required", 0, False

        self.get_logger().info(
            f"WS->ROS set_nav_goals (count={len(waypoints)}, loop={bool(loop)})"
        )
        req = SetNavGoalLL.Request()

        req.lats = [float(wp["lat"]) for wp in waypoints]
        req.lons = [float(wp["lon"]) for wp in waypoints]
        req.yaws_deg = [float(wp.get("yaw_deg", 0.0)) for wp in waypoints]
        req.loop = bool(loop)

        # Keep legacy single-goal fields populated for compatibility.
        req.lat = float(req.lats[0])
        req.lon = float(req.lons[0])
        req.yaw_deg = float(req.yaws_deg[0])

        res = self._call_service(self._nav_set_goal_client, req, self.set_goal_timeout_s)
        if res is None:
            return False, "set_goal_ll timeout", len(waypoints), bool(loop)
        if not res.ok:
            self.get_logger().warning(f"set_nav_goals failed: {res.error}")
        else:
            self.get_logger().info("set_nav_goals ok")
        return bool(res.ok), str(res.error), len(waypoints), bool(loop)

    def cancel_nav_goal(self) -> Tuple[bool, str]:
        req = CancelNavGoal.Request()
        res = self._call_service(self._nav_cancel_goal_client, req, self.request_timeout_s)
        if res is None:
            return False, "cancel_goal timeout"
        return bool(res.ok), str(res.error)

    def brake_nav(self) -> Tuple[bool, str]:
        req = BrakeNav.Request()
        res = self._call_service(self._nav_brake_client, req, self.request_timeout_s)
        if res is None:
            return False, "brake timeout"
        return bool(res.ok), str(res.error)

    def set_manual_mode(self, enabled: bool) -> Tuple[bool, str, bool]:
        req = SetManualMode.Request()
        req.enabled = bool(enabled)
        res = self._call_service(self._nav_set_manual_mode_client, req, self.request_timeout_s)
        if res is None:
            return False, "set_manual_mode timeout", bool(enabled)
        if res.ok:
            self.get_nav_state()
        return bool(res.ok), str(res.error), bool(res.enabled_after)

    def set_manual_cmd(self, linear_x: float, angular_z: float) -> Tuple[bool, str]:
        if not np.isfinite(linear_x) or not np.isfinite(angular_z):
            return False, "invalid manual command values"

        with self._lock:
            manual_enabled = bool(self._manual_control.get("enabled", False))
        if not manual_enabled:
            return False, "manual control is disabled"

        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self._teleop_cmd_pub.publish(cmd)

        with self._lock:
            self._manual_cmd_last_monotonic = time.monotonic()
            self._manual_control["linear_x_cmd"] = float(linear_x)
            self._manual_control["angular_z_cmd"] = float(angular_z)
            self._manual_control["last_cmd_age_s"] = 0.0

        return True, ""

    def get_nav_snapshot(self) -> Tuple[bool, str, Dict[str, Any]]:
        started = time.perf_counter()
        req = GetNavSnapshot.Request()
        res = self._call_service(
            self._nav_snapshot_client, req, self.snapshot_request_timeout_s
        )
        if res is None:
            return False, "nav snapshot timeout", {}
        if not res.ok:
            self.get_logger().warning(f"get_nav_snapshot failed: {res.error}")
            return False, str(res.error), {}

        image_bytes = bytes(res.image_png)
        payload = {
            "op": "nav_snapshot",
            "ok": True,
            "mime": res.mime or "image/png",
            "width": int(res.width),
            "height": int(res.height),
            "frame_id": str(res.frame_id),
            "stamp": {
                "sec": int(res.stamp.sec),
                "nanosec": int(res.stamp.nanosec),
            },
            "layers": {
                "local_costmap": bool(res.layers.local_costmap),
                "global_costmap": bool(res.layers.global_costmap),
                "keepout_mask": bool(res.layers.keepout_mask),
                "footprint": bool(res.layers.footprint),
                "stop_zone": bool(res.layers.stop_zone),
                "scan": bool(res.layers.scan),
                "plan": bool(res.layers.plan),
                "collision_polygons": bool(res.layers.collision_polygons),
                "global_inset": bool(res.layers.global_inset),
            },
            "image_b64": base64.b64encode(image_bytes).decode("ascii"),
            "image_size_bytes": int(len(image_bytes)),
        }
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.get_logger().info(
            f"get_nav_snapshot ok (elapsed_ms={elapsed_ms:.1f}, bytes={len(image_bytes)})"
        )
        return True, "", payload

    def camera_pan(self, angle_deg: float) -> Tuple[bool, str, float]:
        req = CameraPan.Request()
        req.angle_deg = float(angle_deg)
        res = self._call_service(self._camera_pan_client, req, self.request_timeout_s)
        if res is None:
            return False, "camera_pan timeout", 0.0

        applied = float(res.applied_angle_deg)
        if res.ok:
            with self._lock:
                self._camera_status["ok"] = True
                self._camera_status["error"] = ""
                self._camera_status["last_command"] = f"angle:{applied:.1f}"
        else:
            with self._lock:
                self._camera_status["ok"] = False
                self._camera_status["error"] = str(res.error)
        return bool(res.ok), str(res.error), applied

    def camera_zoom_toggle(self) -> Tuple[bool, str]:
        req = Trigger.Request()
        res = self._call_service(
            self._camera_zoom_toggle_client,
            req,
            self.request_timeout_s,
        )
        if res is None:
            return False, "camera_zoom_toggle timeout"
        if res.success:
            with self._lock:
                self._camera_status["ok"] = True
                self._camera_status["error"] = ""
                self._camera_status["last_command"] = "zoom_toggle"
        else:
            with self._lock:
                self._camera_status["ok"] = False
                self._camera_status["error"] = str(res.message)
        return bool(res.success), str(res.message)

    def get_camera_status(self) -> Tuple[bool, str, Dict[str, Any]]:
        req = CameraStatus.Request()
        res = self._call_service(self._camera_status_client, req, self.request_timeout_s)
        if res is None:
            payload = {
                "op": "camera_status",
                "ok": False,
                "error": "camera_status timeout",
                "last_command": "",
                "zoom_in": False,
            }
            return False, payload["error"], payload

        payload = {
            "op": "camera_status",
            "ok": bool(res.ok),
            "error": str(res.error),
            "last_command": str(res.last_command),
            "zoom_in": bool(res.zoom_in),
        }
        with self._lock:
            self._camera_status = {
                "ok": bool(res.ok),
                "error": str(res.error),
                "last_command": str(res.last_command),
                "zoom_in": bool(res.zoom_in),
            }
        return bool(res.ok), str(res.error), payload

    def bootstrap_backend_state(self) -> None:
        self.get_logger().info("Bootstrapping gateway state from backend services...")
        ok_k, err_k = self.get_zones_state()
        if not ok_k and err_k:
            self.get_logger().warning(f"zones bootstrap failed: {err_k}")
        ok_n, err_n = self.get_nav_state()
        if not ok_n and err_n:
            self.get_logger().warning(f"nav bootstrap failed: {err_n}")
        ok_c, err_c, _ = self.get_camera_status()
        if not ok_c and err_c:
            self.get_logger().warning(f"camera bootstrap failed: {err_c}")
        self.get_logger().info("Gateway bootstrap finished")


class WebSocketApi:
    def __init__(self, node: WebZoneServerNode):
        self.node = node

    def _parse_waypoints_from_message(
        self, msg: Dict[str, Any]
    ) -> Tuple[Optional[List[Dict[str, float]]], bool, str]:
        loop = bool(msg.get("loop", False))
        waypoints_raw = msg.get("waypoints")

        if waypoints_raw is None:
            try:
                lat = float(msg["lat"])
                lon = float(msg["lon"])
                yaw_deg = float(msg.get("yaw_deg", 0.0))
            except (KeyError, ValueError, TypeError) as exc:
                return None, False, f"invalid parameters: {exc}"
            if (not np.isfinite(lat)) or (not np.isfinite(lon)) or (not np.isfinite(yaw_deg)):
                return None, False, "lat/lon/yaw_deg must be finite numbers"
            return [{"lat": lat, "lon": lon, "yaw_deg": yaw_deg}], loop, ""

        if not isinstance(waypoints_raw, list) or len(waypoints_raw) == 0:
            return None, False, "waypoints must be a non-empty list"

        waypoints: List[Dict[str, float]] = []
        for idx, item in enumerate(waypoints_raw):
            if not isinstance(item, dict):
                return None, False, f"waypoint[{idx}] must be an object"
            try:
                lat = float(item["lat"])
                lon = float(item["lon"])
                yaw_deg = float(item.get("yaw_deg", 0.0))
            except (KeyError, ValueError, TypeError) as exc:
                return None, False, f"invalid waypoint[{idx}] values: {exc}"
            if (not np.isfinite(lat)) or (not np.isfinite(lon)) or (not np.isfinite(yaw_deg)):
                return None, False, f"waypoint[{idx}] values must be finite"
            waypoints.append({"lat": lat, "lon": lon, "yaw_deg": yaw_deg})

        return waypoints, loop, ""

    async def handle(self, ws: Any, path: Optional[str] = None) -> None:
        _ = path
        self.node.add_client(ws)
        try:
            await ws.send(json.dumps(self.node.snapshot_state()))
            async for raw in ws:
                await self._handle_message(ws, raw)
        finally:
            self.node.remove_client(ws)

    async def _handle_message(self, ws: Any, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            self.node.get_logger().warning("Invalid WS JSON payload received")
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": False,
                        "request": "invalid_json",
                        "error": "invalid json",
                    }
                )
            )
            return

        op = msg.get("op")
        if op != "set_manual_cmd":
            self.node.get_logger().info(f"WS op received: {op}")
        if op == "get_state":
            await ws.send(json.dumps(self.node.snapshot_state()))
            return

        if op == "set_zones_geojson":
            geojson_payload = msg.get("geojson")
            if geojson_payload is None:
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "set_zones_geojson",
                            "error": "geojson field is required",
                            "published": False,
                        }
                    )
                )
                return
            ok, err, published = await asyncio.to_thread(
                self.node.set_zones_geojson, geojson_payload
            )
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_zones_geojson",
                        "error": None if ok else err,
                        "published": bool(published),
                    }
                )
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "load_zones_file":
            ok, err = await asyncio.to_thread(self.node.reload_zones_from_disk)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "load_zones_file",
                        "error": None if ok else err,
                        "published": bool(ok),
                    }
                )
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "set_goal_ll":
            waypoints, loop_enabled, parse_err = self._parse_waypoints_from_message(msg)
            if waypoints is None:
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "set_goal_ll",
                            "error": parse_err,
                        }
                    )
                )
                return
            ok, err, waypoint_count, loop_used = await asyncio.to_thread(
                self.node.set_nav_goals, waypoints, loop_enabled
            )
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_goal_ll",
                        "error": None if ok else err,
                        "waypoint_count": int(waypoint_count),
                        "loop": bool(loop_used),
                    }
                )
            )
            return

        if op == "cancel_goal":
            ok, err = await asyncio.to_thread(self.node.cancel_nav_goal)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "cancel_goal",
                        "error": None if ok else err,
                    }
                )
            )
            return

        if op == "brake":
            ok, err = await asyncio.to_thread(self.node.brake_nav)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "brake",
                        "error": None if ok else err,
                    }
                )
            )
            return

        if op == "set_manual_mode":
            enabled_raw = msg.get("enabled")
            if not isinstance(enabled_raw, bool):
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "set_manual_mode",
                            "error": "enabled must be boolean",
                        }
                    )
                )
                return
            ok, err, enabled_after = await asyncio.to_thread(
                self.node.set_manual_mode,
                enabled_raw,
            )
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_manual_mode",
                        "enabled": enabled_after,
                        "error": None if ok else err,
                    }
                )
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "set_manual_cmd":
            try:
                linear_x = float(msg["linear_x"])
                angular_z = float(msg["angular_z"])
            except (KeyError, ValueError, TypeError) as exc:
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "set_manual_cmd",
                            "error": f"invalid parameters: {exc}",
                        }
                    )
                )
                return
            ok, err = await asyncio.to_thread(
                self.node.set_manual_cmd,
                linear_x,
                angular_z,
            )
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_manual_cmd",
                        "error": None if ok else err,
                    }
                )
            )
            return

        if op == "get_nav_snapshot":
            ok, err, payload = await asyncio.to_thread(self.node.get_nav_snapshot)
            if ok:
                await ws.send(json.dumps(payload))
                return
            await ws.send(
                json.dumps(
                    {
                        "op": "nav_snapshot",
                        "ok": False,
                        "error": err or "snapshot request failed",
                    }
                )
            )
            return

        if op == "camera_pan":
            angle_raw = msg.get("angle")
            try:
                angle = float(angle_raw)
            except (ValueError, TypeError):
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "camera_pan",
                            "error": "angle must be numeric",
                        }
                    )
                )
                return
            if not np.isfinite(angle):
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "camera_pan",
                            "error": "angle must be finite",
                        }
                    )
                )
                return
            ok, err, _ = await asyncio.to_thread(self.node.camera_pan, angle)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "camera_pan",
                        "error": None if ok else err,
                    }
                )
            )
            return

        if op == "camera_zoom_toggle":
            ok, err = await asyncio.to_thread(self.node.camera_zoom_toggle)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "camera_zoom_toggle",
                        "error": None if ok else err,
                    }
                )
            )
            return

        if op == "get_camera_status":
            _, _, payload = await asyncio.to_thread(self.node.get_camera_status)
            await ws.send(json.dumps(payload))
            return

        await ws.send(
            json.dumps(
                {
                    "op": "ack",
                    "ok": False,
                    "request": str(op),
                    "error": "unknown op",
                    "published": False,
                }
            )
        )
        self.node.get_logger().warning(f"Unknown WS op received: {op}")


async def async_main() -> None:
    rclpy.init()
    loop = asyncio.get_running_loop()
    node = WebZoneServerNode(loop)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    await asyncio.to_thread(node.bootstrap_backend_state)

    api = WebSocketApi(node)
    server = await websockets.serve(api.handle, node.ws_host, node.ws_port)
    node.get_logger().info(
        f"WebSocket server listening on ws://{node.ws_host}:{node.ws_port}"
    )

    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        pass
    finally:
        server.close()
        await server.wait_closed()
        executor.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def main() -> None:
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
