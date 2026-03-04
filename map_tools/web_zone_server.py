import asyncio
import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import cv2
import numpy as np
import rclpy
import websockets
import yaml
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix


class WebZoneServerNode(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("web_zone_server")
        self._loop = loop

        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8766)
        self.declare_parameter("fromll_service", "/fromLL")
        self.declare_parameter("global_costmap_service", "/global_costmap/get_costmap")
        self.declare_parameter("mask_topic", "/keepout_filter_mask")
        self.declare_parameter("filter_info_topic", "/costmap_filter_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("gps_broadcast_hz", 1.0)
        self.declare_parameter("brake_topic", "/cmd_vel_safe")
        self.declare_parameter("brake_publish_count", 5)
        self.declare_parameter("brake_publish_interval_s", 0.1)
        self.declare_parameter("zones_file", "")

        self.ws_host = str(self.get_parameter("ws_host").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.fromll_service = str(self.get_parameter("fromll_service").value)
        self.global_costmap_service = str(
            self.get_parameter("global_costmap_service").value
        )
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.filter_info_topic = str(self.get_parameter("filter_info_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.gps_broadcast_hz = float(self.get_parameter("gps_broadcast_hz").value)
        self.brake_topic = str(self.get_parameter("brake_topic").value)
        self.brake_publish_count = max(1, int(self.get_parameter("brake_publish_count").value))
        self.brake_publish_interval_s = max(
            0.0, float(self.get_parameter("brake_publish_interval_s").value)
        )
        self.zones_file = self._resolve_zones_file(str(self.get_parameter("zones_file").value))

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()
        self._zones_ll: List[Dict[str, Any]] = []
        self._zones_xy: List[Dict[str, Any]] = []
        self._mask_ready = False
        self._mask_source = "none"
        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_gps_broadcast_monotonic: Optional[float] = None

        self._fromll_client = self.create_client(FromLL, self.fromll_service)
        self._global_costmap_client = self.create_client(
            GetCostmap, self.global_costmap_service
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE

        self._mask_pub = self.create_publisher(OccupancyGrid, self.mask_topic, latched_qos)
        self._filter_info_pub = self.create_publisher(
            CostmapFilterInfo, self.filter_info_topic, latched_qos
        )
        self._brake_pub = self.create_publisher(Twist, self.brake_topic, 10)
        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, 10
        )

        # Nav2 NavigateToPose action client
        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._current_goal_handle = None

        self._publish_filter_info()

    def _resolve_zones_file(self, configured: str) -> Path:
        if configured:
            return Path(configured)
        return Path.cwd() / "config" / "no_go_zones.yaml"

    def add_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.add(ws)

    def remove_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.discard(ws)

    def snapshot_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "op": "state",
                "ok": True,
                "frame_id": self.map_frame,
                "zones": list(self._zones_ll),
                "mask_ready": self._mask_ready,
                "mask_source": self._mask_source,
                "robot_pose": self._last_robot_pose,
            }

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not np.isfinite(msg.latitude) or not np.isfinite(msg.longitude):
            return
        pose = {
            "lat": float(msg.latitude),
            "lon": float(msg.longitude),
        }
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

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
        text = json.dumps(payload)
        with self._lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        failed: List[Any] = []
        for ws in clients:
            try:
                await ws.send(text)
            except Exception:
                failed.append(ws)
        if failed:
            with self._lock:
                for ws in failed:
                    self._ws_clients.discard(ws)

    def _publish_filter_info(self) -> None:
        msg = CostmapFilterInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.type = 0
        msg.filter_mask_topic = self.mask_topic
        msg.base = 0.0
        msg.multiplier = 1.0
        self._filter_info_pub.publish(msg)

    def _wait_for_future(self, future: Any, timeout_sec: float) -> Optional[Any]:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return future.result()
            if (time.monotonic() - start) >= timeout_sec:
                return None
            time.sleep(0.01)
        return None

    def _call_from_ll(self, lat: float, lon: float) -> Optional[Tuple[float, float, float]]:
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
        future = self._fromll_client.call_async(req)
        res = self._wait_for_future(future, timeout_sec=2.5)
        if res is None:
            return None
        return (float(res.map_point.x), float(res.map_point.y), float(res.map_point.z))

    def _yaw_to_quaternion(self, yaw_deg: float) -> Quaternion:
        """Convert yaw angle in degrees to quaternion (z-axis rotation only)."""
        yaw_rad = math.radians(yaw_deg)
        half_yaw = yaw_rad / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def send_nav2_goal(self, lat: float, lon: float, yaw_deg: float = 0.0) -> Tuple[bool, str]:
        """Send a NavigateToPose goal to Nav2."""
        # Convert lat/lon to map frame
        converted = self._call_from_ll(lat, lon)
        if converted is None:
            return False, "fromLL conversion failed"
        x, y, _ = converted

        # Build PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(yaw_deg)

        # Send goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Wait for action server
        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            return False, "NavigateToPose action server not available"

        future = self._nav2_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future, timeout_sec=5.0)
        if goal_handle is None:
            return False, "failed to send goal"

        if not goal_handle.accepted:
            return False, "goal rejected by Nav2"

        self._current_goal_handle = goal_handle
        return True, "goal accepted"

    def cancel_current_goal(self) -> Tuple[bool, str]:
        """Cancel the current Nav2 goal."""
        if self._current_goal_handle is None:
            return False, "no active goal"

        future = self._current_goal_handle.cancel_goal_async()
        result = self._wait_for_future(future, timeout_sec=2.0)
        if result is None:
            return False, "timeout cancelling goal"

        self._current_goal_handle = None
        return True, "cancelled"

    def apply_brake(self) -> Tuple[bool, str]:
        """Cancel the current goal if present and publish zero velocity commands."""
        cancel_ok = True
        cancel_msg = "no active goal"
        if self._current_goal_handle is not None:
            cancel_ok, cancel_msg = self.cancel_current_goal()

        stop_cmd = Twist()
        for index in range(self.brake_publish_count):
            self._brake_pub.publish(stop_cmd)
            if index + 1 < self.brake_publish_count and self.brake_publish_interval_s > 0.0:
                time.sleep(self.brake_publish_interval_s)

        self.get_logger().warning(
            f"Brake command sent on {self.brake_topic} "
            f"({self.brake_publish_count} messages, cancel={cancel_msg})"
        )

        if cancel_ok:
            return True, "brake applied"
        return False, f"brake applied, but goal cancel failed: {cancel_msg}"

    def _get_global_costmap(self) -> Optional[Any]:
        if not self._global_costmap_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f"Service {self.global_costmap_service} not available"
            )
            return None
        future = self._global_costmap_client.call_async(GetCostmap.Request())
        res = self._wait_for_future(future, timeout_sec=2.5)
        if res is None:
            self.get_logger().warning(
                f"Timeout calling {self.global_costmap_service}"
            )
            return None
        return res.map

    def _xy_to_grid(
        self, x: float, y: float, origin_x: float, origin_y: float, resolution: float
    ) -> Tuple[int, int]:
        col = int(np.floor((x - origin_x) / resolution))
        row = int(np.floor((y - origin_y) / resolution))
        return col, row

    def _rasterize_mask(self, zones_xy: List[Dict[str, Any]], map_msg: Any) -> OccupancyGrid:
        md = map_msg.metadata
        width = int(md.size_x)
        height = int(md.size_y)
        resolution = float(md.resolution)
        origin_x = float(md.origin.position.x)
        origin_y = float(md.origin.position.y)

        mask = np.zeros((height, width), dtype=np.uint8)

        for zone in zones_xy:
            if zone.get("enabled", True) is False:
                continue
            polygon_xy = zone.get("polygon_xy", [])
            if len(polygon_xy) < 3:
                continue

            pts: List[List[int]] = []
            for p in polygon_xy:
                col, row = self._xy_to_grid(
                    float(p["x"]),
                    float(p["y"]),
                    origin_x,
                    origin_y,
                    resolution,
                )
                if col < 0:
                    col = 0
                elif col >= width:
                    col = width - 1
                if row < 0:
                    row = 0
                elif row >= height:
                    row = height - 1
                img_row = height - 1 - row
                pts.append([col, img_row])

            if len(pts) >= 3:
                arr = np.array([pts], dtype=np.int32)
                cv2.fillPoly(mask, arr, 100)

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = map_msg.header.frame_id or self.map_frame
        out.info.map_load_time = out.header.stamp
        out.info.resolution = md.resolution
        out.info.width = md.size_x
        out.info.height = md.size_y
        out.info.origin = md.origin
        # OccupancyGrid expects row-major from map origin (bottom-left),
        # while OpenCV raster is top-left based.
        out.data = np.flipud(mask).reshape(-1).astype(np.int8).tolist()
        return out

    def _convert_zones_to_xy(self, zones: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for zone in zones:
            zone_id = str(zone.get("id", ""))
            ztype = str(zone.get("type", "no_go"))
            enabled = bool(zone.get("enabled", True))
            polygon_ll = zone.get("polygon", [])
            if not isinstance(polygon_ll, list) or len(polygon_ll) < 3:
                self.get_logger().warning(f"Skipping zone '{zone_id}': invalid polygon")
                continue

            polygon_xy: List[Dict[str, float]] = []
            failed = False
            for vertex in polygon_ll:
                try:
                    lat = float(vertex["lat"])
                    lon = float(vertex["lon"])
                except Exception:
                    failed = True
                    break

                converted = self._call_from_ll(lat, lon)
                if converted is None:
                    failed = True
                    break
                x, y, _ = converted
                polygon_xy.append({"x": x, "y": y})

            if failed or len(polygon_xy) < 3:
                self.get_logger().warning(
                    f"Skipping zone '{zone_id}': fromLL conversion failed"
                )
                continue

            out.append(
                {
                    "id": zone_id,
                    "type": ztype,
                    "enabled": enabled,
                    "polygon_xy": polygon_xy,
                }
            )
        return out

    def set_zones_ll_and_publish(self, zones: List[Dict[str, Any]]) -> Tuple[bool, str, bool]:
        if not self._fromll_client.wait_for_service(timeout_sec=3.0):
            return False, f"Service {self.fromll_service} not available", False

        map_msg = self._get_global_costmap()
        if map_msg is None:
            return False, f"Service {self.global_costmap_service} failed", False

        zones_xy = self._convert_zones_to_xy(zones)
        occ = self._rasterize_mask(zones_xy, map_msg)

        self._publish_filter_info()
        self._mask_pub.publish(occ)

        with self._lock:
            self._zones_ll = zones
            self._zones_xy = zones_xy
            self._mask_ready = True
            self._mask_source = "global_costmap"

        self._save_zones_to_disk()
        return True, "ok", True

    def _save_zones_to_disk(self) -> None:
        data = {"frame_id": self.map_frame, "zones": self._zones_ll}
        try:
            self.zones_file.parent.mkdir(parents=True, exist_ok=True)
            with self.zones_file.open("w", encoding="utf-8") as handle:
                yaml.safe_dump(data, handle, sort_keys=False)
        except Exception as exc:
            self.get_logger().warning(f"Failed to save zones file: {exc}")

    def _load_zones_from_disk(self) -> None:
        if not self.zones_file.exists():
            self.get_logger().info(f"No zones file found at {str(self.zones_file)}")
            return
        try:
            with self.zones_file.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            zones = data.get("zones", [])
            if not isinstance(zones, list):
                self.get_logger().warning("Zones file is invalid, expected list")
                return
            ok, err, _ = self.set_zones_ll_and_publish(zones)
            if ok:
                self.get_logger().info(
                    f"Loaded {len(zones)} zones from {str(self.zones_file)}"
                )
            else:
                self.get_logger().warning(f"Failed to load zones from disk: {err}")
        except Exception as exc:
            self.get_logger().warning(
                f"Failed reading zones file {str(self.zones_file)}: {exc}"
            )

    def load_initial_zones(self) -> None:
        self._load_zones_from_disk()


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
            await ws.send(
                json.dumps({"op": "ack", "ok": False, "request": "invalid_json", "error": "invalid json"})
            )
            return

        op = msg.get("op")
        if op == "get_state":
            await ws.send(json.dumps(self.node.snapshot_state()))
            return

        if op == "set_goal_ll":
            try:
                lat = float(msg["lat"])
                lon = float(msg["lon"])
                yaw = float(msg.get("yaw_deg", 0.0))
            except (KeyError, ValueError, TypeError) as e:
                await ws.send(
                    json.dumps({
                        "op": "ack",
                        "ok": False,
                        "request": "set_goal_ll",
                        "error": f"invalid parameters: {str(e)}"
                    })
                )
                return
            ok, err = await asyncio.to_thread(self.node.send_nav2_goal, lat, lon, yaw)
            await ws.send(
                json.dumps({
                    "op": "ack",
                    "ok": ok,
                    "request": "set_goal_ll",
                    "error": None if ok else err
                })
            )
            return

        if op == "cancel_goal":
            ok, err = await asyncio.to_thread(self.node.cancel_current_goal)
            await ws.send(
                json.dumps({
                    "op": "ack",
                    "ok": ok,
                    "request": "cancel_goal",
                    "error": None if ok else err
                })
            )
            return

        if op == "brake":
            ok, err = await asyncio.to_thread(self.node.apply_brake)
            await ws.send(
                json.dumps({
                    "op": "ack",
                    "ok": ok,
                    "request": "brake",
                    "error": None if ok else err,
                })
            )
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

            ok, err, published = await asyncio.to_thread(self.node.set_zones_ll_and_publish, zones)
            await ws.send(
                json.dumps(
                    {
                        "op": "ack",
                        "ok": ok,
                        "request": "set_zones_ll",
                        "error": None if ok else err,
                        "published": published,
                    }
                )
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
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


async def async_main() -> None:
    rclpy.init()
    loop = asyncio.get_running_loop()
    node = WebZoneServerNode(loop)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    await asyncio.to_thread(node.load_initial_zones)

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
        rclpy.shutdown()


def main() -> None:
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
