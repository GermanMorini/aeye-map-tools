import asyncio
import base64
import json
import threading
import time
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
import websockets
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from navegacion_gps_interfaces.msg import NavTelemetry, NoGoPoint, NoGoZone
from navegacion_gps_interfaces.srv import (
    BrakeNav,
    CancelNavGoal,
    GetKeepoutState,
    GetNavSnapshot,
    GetNavState,
    SetKeepoutZones,
    SetManualCmd,
    SetManualMode,
    SetNavGoalLL,
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

        self.declare_parameter("keepout_set_zones_service", "/keepout_manager/set_zones")
        self.declare_parameter("keepout_get_state_service", "/keepout_manager/get_state")

        self.declare_parameter("nav_set_goal_service", "/nav_command_server/set_goal_ll")
        self.declare_parameter("nav_cancel_goal_service", "/nav_command_server/cancel_goal")
        self.declare_parameter("nav_brake_service", "/nav_command_server/brake")
        self.declare_parameter("nav_set_manual_mode_service", "/nav_command_server/set_manual_mode")
        self.declare_parameter("nav_set_manual_cmd_service", "/nav_command_server/set_manual_cmd")
        self.declare_parameter("nav_get_state_service", "/nav_command_server/get_state")

        self.declare_parameter("nav_snapshot_service", "/nav_snapshot_server/get_nav_snapshot")
        self.declare_parameter("nav_telemetry_topic", "/nav_command_server/telemetry")

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

        self.keepout_set_zones_service = str(
            self.get_parameter("keepout_set_zones_service").value
        )
        self.keepout_get_state_service = str(
            self.get_parameter("keepout_get_state_service").value
        )

        self.nav_set_goal_service = str(self.get_parameter("nav_set_goal_service").value)
        self.nav_cancel_goal_service = str(
            self.get_parameter("nav_cancel_goal_service").value
        )
        self.nav_brake_service = str(self.get_parameter("nav_brake_service").value)
        self.nav_set_manual_mode_service = str(
            self.get_parameter("nav_set_manual_mode_service").value
        )
        self.nav_set_manual_cmd_service = str(
            self.get_parameter("nav_set_manual_cmd_service").value
        )
        self.nav_get_state_service = str(self.get_parameter("nav_get_state_service").value)

        self.nav_snapshot_service = str(self.get_parameter("nav_snapshot_service").value)
        self.nav_telemetry_topic = str(self.get_parameter("nav_telemetry_topic").value)

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()

        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_gps_broadcast_monotonic: Optional[float] = None

        self._zones: List[Dict[str, Any]] = []
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

        self._manual_cmd_last_monotonic: Optional[float] = None

        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, 10
        )
        self._nav_telemetry_sub = self.create_subscription(
            NavTelemetry, self.nav_telemetry_topic, self._on_nav_telemetry, 10
        )

        self._keepout_set_zones_client = self.create_client(
            SetKeepoutZones, self.keepout_set_zones_service
        )
        self._keepout_get_state_client = self.create_client(
            GetKeepoutState, self.keepout_get_state_service
        )
        self._nav_set_goal_client = self.create_client(SetNavGoalLL, self.nav_set_goal_service)
        self._nav_cancel_goal_client = self.create_client(
            CancelNavGoal, self.nav_cancel_goal_service
        )
        self._nav_brake_client = self.create_client(BrakeNav, self.nav_brake_service)
        self._nav_set_manual_mode_client = self.create_client(
            SetManualMode, self.nav_set_manual_mode_service
        )
        self._nav_set_manual_cmd_client = self.create_client(
            SetManualCmd, self.nav_set_manual_cmd_service
        )
        self._nav_get_state_client = self.create_client(GetNavState, self.nav_get_state_service)
        self._nav_snapshot_client = self.create_client(GetNavSnapshot, self.nav_snapshot_service)
        self.get_logger().info(
            "Web gateway ready "
            f"(ws={self.ws_host}:{self.ws_port}, keepout_set={self.keepout_set_zones_service}, "
            f"goal_set={self.nav_set_goal_service}, snapshot={self.nav_snapshot_service})"
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
                "mask_ready": bool(self._mask_ready),
                "mask_source": str(self._mask_source),
                "robot_pose": self._last_robot_pose,
                "cmd_vel_safe": dict(self._cmd_vel_safe),
                "manual_control": dict(self._manual_control),
                "goal_active": bool(self._goal_active),
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

    def _zone_dict_to_msg(self, zone: Dict[str, Any]) -> NoGoZone:
        out = NoGoZone()
        out.id = str(zone.get("id", ""))
        out.type = str(zone.get("type", "no_go"))
        out.enabled = bool(zone.get("enabled", True))

        points: List[NoGoPoint] = []
        polygon = zone.get("polygon", [])
        if isinstance(polygon, list):
            for v in polygon:
                try:
                    lat = float(v["lat"])
                    lon = float(v["lon"])
                except Exception:
                    continue
                p = NoGoPoint()
                p.lat = lat
                p.lon = lon
                points.append(p)
        out.polygon = points
        return out

    def _zone_msg_to_dict(self, zone_msg: NoGoZone) -> Dict[str, Any]:
        polygon = []
        for p in zone_msg.polygon:
            polygon.append({"lat": float(p.lat), "lon": float(p.lon)})
        return {
            "id": str(zone_msg.id),
            "type": str(zone_msg.type),
            "enabled": bool(zone_msg.enabled),
            "polygon": polygon,
        }

    def _update_keepout_state(self, response: GetKeepoutState.Response) -> None:
        with self._lock:
            self._zones = [self._zone_msg_to_dict(z) for z in response.zones]
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

    def get_keepout_state(self) -> Tuple[bool, str]:
        req = GetKeepoutState.Request()
        res = self._call_service(self._keepout_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "keepout get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_keepout_state(res)
        return True, ""

    def set_keepout_zones(self, zones: List[Dict[str, Any]]) -> Tuple[bool, str, bool]:
        self.get_logger().info(f"WS->ROS set_keepout_zones (zones={len(zones)})")
        req = SetKeepoutZones.Request()
        req.zones = [self._zone_dict_to_msg(z) for z in zones]
        res = self._call_service(self._keepout_set_zones_client, req, self.set_zones_timeout_s)
        if res is None:
            return False, "keepout set_zones timeout", False
        if not res.ok:
            self.get_logger().warning(f"set_keepout_zones failed: {res.error}")
            return False, str(res.error), bool(res.published)
        self.get_keepout_state()
        self.get_logger().info(f"set_keepout_zones ok (published={bool(res.published)})")
        return True, "", bool(res.published)

    def get_nav_state(self) -> Tuple[bool, str]:
        req = GetNavState.Request()
        res = self._call_service(self._nav_get_state_client, req, self.request_timeout_s)
        if res is None:
            return False, "nav get_state timeout"
        if not res.ok:
            return False, str(res.error)
        self._update_nav_state(res)
        return True, ""

    def set_nav_goal(self, lat: float, lon: float, yaw_deg: float) -> Tuple[bool, str]:
        self.get_logger().info(
            f"WS->ROS set_nav_goal (lat={lat:.8f}, lon={lon:.8f}, yaw={yaw_deg:.2f})"
        )
        req = SetNavGoalLL.Request()
        req.lat = float(lat)
        req.lon = float(lon)
        req.yaw_deg = float(yaw_deg)
        res = self._call_service(self._nav_set_goal_client, req, self.set_goal_timeout_s)
        if res is None:
            return False, "set_goal_ll timeout"
        if not res.ok:
            self.get_logger().warning(f"set_nav_goal failed: {res.error}")
        else:
            self.get_logger().info("set_nav_goal ok")
        return bool(res.ok), str(res.error)

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
        req = SetManualCmd.Request()
        req.linear_x = float(linear_x)
        req.angular_z = float(angular_z)
        res = self._call_service(self._nav_set_manual_cmd_client, req, self.request_timeout_s)
        if res is None:
            return False, "set_manual_cmd timeout"

        with self._lock:
            self._manual_cmd_last_monotonic = time.monotonic()
            self._manual_control["linear_x_cmd"] = float(linear_x)
            self._manual_control["angular_z_cmd"] = float(angular_z)
            self._manual_control["last_cmd_age_s"] = 0.0

        return bool(res.ok), str(res.error)

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

    def bootstrap_backend_state(self) -> None:
        self.get_logger().info("Bootstrapping gateway state from backend services...")
        ok_k, err_k = self.get_keepout_state()
        if not ok_k and err_k:
            self.get_logger().warning(f"keepout bootstrap failed: {err_k}")
        ok_n, err_n = self.get_nav_state()
        if not ok_n and err_n:
            self.get_logger().warning(f"nav bootstrap failed: {err_n}")
        self.get_logger().info("Gateway bootstrap finished")


class WebSocketApi:
    def __init__(self, node: WebZoneServerNode):
        self.node = node

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

        if op == "set_zones_ll":
            zones = msg.get("zones", [])
            if not isinstance(zones, list):
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "set_zones_ll",
                            "error": "zones must be a list",
                            "published": False,
                        }
                    )
                )
                return
            ok, err, published = await asyncio.to_thread(
                self.node.set_keepout_zones, zones
            )
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_zones_ll",
                        "error": None if ok else err,
                        "published": bool(published),
                    }
                )
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "set_goal_ll":
            try:
                lat = float(msg["lat"])
                lon = float(msg["lon"])
                yaw = float(msg.get("yaw_deg", 0.0))
            except (KeyError, ValueError, TypeError) as exc:
                await ws.send(
                    json.dumps(
                        {
                            "op": "ack",
                            "ok": False,
                            "request": "set_goal_ll",
                            "error": f"invalid parameters: {exc}",
                        }
                    )
                )
                return
            ok, err = await asyncio.to_thread(self.node.set_nav_goal, lat, lon, yaw)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_goal_ll",
                        "error": None if ok else err,
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
