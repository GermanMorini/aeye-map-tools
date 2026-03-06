import asyncio
import base64
import copy
import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
import websockets
import yaml
import cv2
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PolygonStamped, Pose, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path as NavPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_localization.srv import FromLL
from sensor_msgs.msg import LaserScan, NavSatFix
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from .mask_utils import exponential_gradient_from_core, rasterize_polygons_core


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
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("nav_telemetry_hz", 5.0)
        self.declare_parameter("brake_publish_count", 5)
        self.declare_parameter("brake_publish_interval_s", 0.1)
        self.declare_parameter("zones_file", "")
        self.declare_parameter("degrade_enabled", True)
        self.declare_parameter("degrade_radius_m", 2.0)
        self.declare_parameter("degrade_edge_cost", 12)
        self.declare_parameter("degrade_min_cost", 1)
        self.declare_parameter("degrade_use_l2", True)
        self.declare_parameter("use_fixed_mask_grid", True)
        self.declare_parameter("mask_origin_x", -150.0)
        self.declare_parameter("mask_origin_y", -150.0)
        self.declare_parameter("mask_width", 3000)
        self.declare_parameter("mask_height", 3000)
        self.declare_parameter("mask_resolution", 0.1)
        self.declare_parameter("snapshot_extent_m", 30.0)
        self.declare_parameter("snapshot_size_px", 512)
        self.declare_parameter("snapshot_global_inset_px", 160)
        self.declare_parameter("snapshot_timeout_ms", 500)
        self.declare_parameter("local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("local_footprint_topic", "/local_costmap/published_footprint")
        self.declare_parameter("stop_zone_topic", "/stop_zone")
        self.declare_parameter("collision_polygons_topic", "/collision_monitor/polygons")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("plan_topic", "/plan")

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
        self.cmd_vel_safe_topic = str(self.get_parameter("cmd_vel_safe_topic").value)
        self.nav_telemetry_hz = float(self.get_parameter("nav_telemetry_hz").value)
        self.brake_publish_count = max(1, int(self.get_parameter("brake_publish_count").value))
        self.brake_publish_interval_s = max(
            0.0, float(self.get_parameter("brake_publish_interval_s").value)
        )
        self.zones_file = self._resolve_zones_file(str(self.get_parameter("zones_file").value))
        self.degrade_enabled = bool(self.get_parameter("degrade_enabled").value)
        self.degrade_radius_m = float(self.get_parameter("degrade_radius_m").value)
        self.degrade_edge_cost = int(self.get_parameter("degrade_edge_cost").value)
        self.degrade_min_cost = int(self.get_parameter("degrade_min_cost").value)
        self.degrade_use_l2 = bool(self.get_parameter("degrade_use_l2").value)
        self.use_fixed_mask_grid = bool(self.get_parameter("use_fixed_mask_grid").value)
        self.mask_origin_x = float(self.get_parameter("mask_origin_x").value)
        self.mask_origin_y = float(self.get_parameter("mask_origin_y").value)
        self.mask_width = int(self.get_parameter("mask_width").value)
        self.mask_height = int(self.get_parameter("mask_height").value)
        self.mask_resolution = float(self.get_parameter("mask_resolution").value)
        self.snapshot_extent_m = float(self.get_parameter("snapshot_extent_m").value)
        self.snapshot_size_px = int(self.get_parameter("snapshot_size_px").value)
        self.snapshot_global_inset_px = int(
            self.get_parameter("snapshot_global_inset_px").value
        )
        self.snapshot_timeout_ms = int(self.get_parameter("snapshot_timeout_ms").value)
        self.local_costmap_topic = str(self.get_parameter("local_costmap_topic").value)
        self.global_costmap_topic = str(self.get_parameter("global_costmap_topic").value)
        self.local_footprint_topic = str(
            self.get_parameter("local_footprint_topic").value
        )
        self.stop_zone_topic = str(self.get_parameter("stop_zone_topic").value)
        self.collision_polygons_topic = str(
            self.get_parameter("collision_polygons_topic").value
        )
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self._sanitize_degrade_params()
        self._sanitize_mask_grid_params()
        self._sanitize_snapshot_params()
        self._sanitize_telemetry_params()

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()
        self._zones_ll: List[Dict[str, Any]] = []
        self._zones_xy: List[Dict[str, Any]] = []
        self._fixed_mask_info: Optional[MapMetaData] = None
        self._mask_ready = False
        self._mask_source = "none"
        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_gps_broadcast_monotonic: Optional[float] = None
        self._last_local_costmap: Optional[OccupancyGrid] = None
        self._last_global_costmap: Optional[OccupancyGrid] = None
        self._last_keepout_mask: Optional[OccupancyGrid] = None
        self._last_local_footprint: Optional[PolygonStamped] = None
        self._last_stop_zone: Optional[PolygonStamped] = None
        self._last_collision_polygons: Optional[MarkerArray] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_plan: Optional[NavPath] = None
        self._last_cmd_vel_safe: Optional[Twist] = None
        self._last_nav_telemetry_monotonic: Optional[float] = None

        self._fromll_client = self.create_client(FromLL, self.fromll_service)
        self._global_costmap_client = self.create_client(
            GetCostmap, self.global_costmap_service
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
        self._cmd_vel_safe_sub = self.create_subscription(
            Twist, self.cmd_vel_safe_topic, self._on_cmd_vel_safe, 10
        )

        costmap_qos = QoSProfile(depth=1)
        costmap_qos.reliability = ReliabilityPolicy.RELIABLE
        costmap_qos.durability = DurabilityPolicy.VOLATILE
        costmap_qos.history = HistoryPolicy.KEEP_LAST

        scan_qos = QoSProfile(depth=1)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE
        scan_qos.history = HistoryPolicy.KEEP_LAST

        self._local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self._on_local_costmap,
            costmap_qos,
        )
        self._global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            self._on_global_costmap,
            costmap_qos,
        )
        self._keepout_mask_sub = self.create_subscription(
            OccupancyGrid,
            self.mask_topic,
            self._on_keepout_mask,
            costmap_qos,
        )
        self._local_footprint_sub = self.create_subscription(
            PolygonStamped,
            self.local_footprint_topic,
            self._on_local_footprint,
            costmap_qos,
        )
        self._stop_zone_sub = self.create_subscription(
            PolygonStamped,
            self.stop_zone_topic,
            self._on_stop_zone,
            costmap_qos,
        )
        self._collision_polygons_sub = self.create_subscription(
            MarkerArray,
            self.collision_polygons_topic,
            self._on_collision_polygons,
            costmap_qos,
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            scan_qos,
        )
        self._plan_sub = self.create_subscription(
            NavPath,
            self.plan_topic,
            self._on_plan,
            costmap_qos,
        )

        # Nav2 NavigateToPose action client
        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._current_goal_handle = None

        if self.use_fixed_mask_grid:
            self._fixed_mask_info = self._build_fixed_mask_metadata()

        self._publish_filter_info()

    def _sanitize_degrade_params(self) -> None:
        if self.degrade_radius_m < 0.0:
            self.get_logger().warning(
                f"degrade_radius_m={self.degrade_radius_m} is invalid; forcing 0.0"
            )
            self.degrade_radius_m = 0.0

        if self.degrade_edge_cost < 1 or self.degrade_edge_cost > 99:
            self.get_logger().warning(
                f"degrade_edge_cost={self.degrade_edge_cost} is out of range [1,99]; clamping"
            )
            self.degrade_edge_cost = max(1, min(99, self.degrade_edge_cost))

        if self.degrade_min_cost < 1 or self.degrade_min_cost > 99:
            self.get_logger().warning(
                f"degrade_min_cost={self.degrade_min_cost} is out of range [1,99]; clamping"
            )
            self.degrade_min_cost = max(1, min(99, self.degrade_min_cost))

        self.get_logger().info(
            "Keepout degrade params: "
            f"enabled={self.degrade_enabled}, "
            f"radius_m={self.degrade_radius_m:.3f}, "
            f"edge_cost={self.degrade_edge_cost}, "
            f"min_cost={self.degrade_min_cost}, "
            f"distance={'L2' if self.degrade_use_l2 else 'L1'}"
        )

    def _sanitize_mask_grid_params(self) -> None:
        if self.mask_width <= 0:
            self.get_logger().warning(
                f"mask_width={self.mask_width} is invalid; forcing 3000"
            )
            self.mask_width = 3000
        if self.mask_height <= 0:
            self.get_logger().warning(
                f"mask_height={self.mask_height} is invalid; forcing 3000"
            )
            self.mask_height = 3000
        if self.mask_resolution <= 0.0 or not np.isfinite(self.mask_resolution):
            self.get_logger().warning(
                f"mask_resolution={self.mask_resolution} is invalid; forcing 0.1"
            )
            self.mask_resolution = 0.1
        if not np.isfinite(self.mask_origin_x):
            self.get_logger().warning(
                f"mask_origin_x={self.mask_origin_x} is invalid; forcing -150.0"
            )
            self.mask_origin_x = -150.0
        if not np.isfinite(self.mask_origin_y):
            self.get_logger().warning(
                f"mask_origin_y={self.mask_origin_y} is invalid; forcing -150.0"
            )
            self.mask_origin_y = -150.0

        if self.use_fixed_mask_grid:
            size_x_m = self.mask_width * self.mask_resolution
            size_y_m = self.mask_height * self.mask_resolution
            self.get_logger().info(
                "Fixed mask grid enabled: "
                f"origin=({self.mask_origin_x:.3f},{self.mask_origin_y:.3f}), "
                f"size=({self.mask_width}x{self.mask_height}), "
                f"resolution={self.mask_resolution:.3f}m "
                f"({size_x_m:.1f}m x {size_y_m:.1f}m)"
            )
        else:
            self.get_logger().info(
                f"Fixed mask grid disabled; using {self.global_costmap_service}"
            )

    def _sanitize_snapshot_params(self) -> None:
        if self.snapshot_extent_m <= 1.0 or not np.isfinite(self.snapshot_extent_m):
            self.get_logger().warning(
                f"snapshot_extent_m={self.snapshot_extent_m} is invalid; forcing 30.0"
            )
            self.snapshot_extent_m = 30.0
        if self.snapshot_size_px < 128:
            self.get_logger().warning(
                f"snapshot_size_px={self.snapshot_size_px} is invalid; forcing 512"
            )
            self.snapshot_size_px = 512
        if self.snapshot_global_inset_px < 64:
            self.get_logger().warning(
                f"snapshot_global_inset_px={self.snapshot_global_inset_px} is invalid; forcing 160"
            )
            self.snapshot_global_inset_px = 160
        if self.snapshot_timeout_ms <= 0:
            self.get_logger().warning(
                f"snapshot_timeout_ms={self.snapshot_timeout_ms} is invalid; forcing 500"
            )
            self.snapshot_timeout_ms = 500

        self.get_logger().info(
            "Snapshot params: "
            f"extent_m={self.snapshot_extent_m:.2f}, "
            f"size_px={self.snapshot_size_px}, "
            f"inset_px={self.snapshot_global_inset_px}, "
            f"timeout_ms={self.snapshot_timeout_ms}"
        )

    def _sanitize_telemetry_params(self) -> None:
        if self.nav_telemetry_hz <= 0.0 or not np.isfinite(self.nav_telemetry_hz):
            self.get_logger().warning(
                f"nav_telemetry_hz={self.nav_telemetry_hz} is invalid; forcing 5.0"
            )
            self.nav_telemetry_hz = 5.0
        self.get_logger().info(
            "Navigation telemetry params: "
            f"cmd_vel_topic={self.cmd_vel_safe_topic}, "
            f"broadcast_hz={self.nav_telemetry_hz:.2f}"
        )

    def _build_fixed_mask_metadata(self) -> MapMetaData:
        info = MapMetaData()
        info.map_load_time = self.get_clock().now().to_msg()
        info.resolution = float(self.mask_resolution)
        info.width = int(self.mask_width)
        info.height = int(self.mask_height)

        origin = Pose()
        origin.position.x = float(self.mask_origin_x)
        origin.position.y = float(self.mask_origin_y)
        origin.position.z = 0.0
        origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        info.origin = origin
        return info

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
            cmd_vel_safe = self._cmd_vel_safe_payload_locked()
            return {
                "op": "state",
                "ok": True,
                "frame_id": self.map_frame,
                "zones": list(self._zones_ll),
                "mask_ready": self._mask_ready,
                "mask_source": self._mask_source,
                "robot_pose": self._last_robot_pose,
                "cmd_vel_safe": cmd_vel_safe,
            }

    def _cmd_vel_safe_payload_locked(self) -> Dict[str, Any]:
        if self._last_cmd_vel_safe is None:
            return {
                "available": False,
                "linear_x": 0.0,
                "angular_z": 0.0,
            }
        msg = self._last_cmd_vel_safe
        return {
            "available": True,
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
        }

    def _build_nav_telemetry_payload(self) -> Dict[str, Any]:
        with self._lock:
            cmd_vel_safe = self._cmd_vel_safe_payload_locked()
        return {
            "op": "nav_telemetry",
            "cmd_vel_safe": cmd_vel_safe,
        }

    def _broadcast_nav_telemetry(self, force: bool = False) -> None:
        now = time.monotonic()
        min_interval = 1.0 / max(0.1, float(self.nav_telemetry_hz))
        with self._lock:
            last_sent = self._last_nav_telemetry_monotonic
            if (not force) and (last_sent is not None) and ((now - last_sent) < min_interval):
                return
            self._last_nav_telemetry_monotonic = now
        payload = self._build_nav_telemetry_payload()
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._last_local_costmap = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._last_global_costmap = msg

    def _on_keepout_mask(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._last_keepout_mask = msg

    def _on_local_footprint(self, msg: PolygonStamped) -> None:
        with self._lock:
            self._last_local_footprint = msg

    def _on_stop_zone(self, msg: PolygonStamped) -> None:
        with self._lock:
            self._last_stop_zone = msg

    def _on_collision_polygons(self, msg: MarkerArray) -> None:
        with self._lock:
            self._last_collision_polygons = msg

    def _on_scan(self, msg: LaserScan) -> None:
        with self._lock:
            self._last_scan = msg

    def _on_plan(self, msg: NavPath) -> None:
        with self._lock:
            self._last_plan = msg

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        with self._lock:
            self._last_cmd_vel_safe = msg
        self._broadcast_nav_telemetry(force=False)

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

    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
        cosy_cosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z))
        return float(math.atan2(siny_cosp, cosy_cosp))

    def _lookup_transform_2d(
        self, target_frame: str, source_frame: str, timeout_sec: float = 0.1
    ) -> Optional[Tuple[float, float, float, float]]:
        if target_frame == source_frame:
            return (1.0, 0.0, 0.0, 0.0)
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException:
            return None
        yaw = self._quaternion_to_yaw(tf.transform.rotation)
        c = float(math.cos(yaw))
        s = float(math.sin(yaw))
        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        return (c, s, tx, ty)

    def _lookup_robot_pose_in_frame(
        self, target_frame: str, timeout_sec: float = 0.1
    ) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                "base_footprint",
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException:
            return None
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        yaw = self._quaternion_to_yaw(tf.transform.rotation)
        return (x, y, yaw)

    def _apply_transform_2d(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        c: float,
        s: float,
        tx: float,
        ty: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        out_x = (c * xs) - (s * ys) + tx
        out_y = (s * xs) + (c * ys) + ty
        return out_x, out_y

    def _occupancy_to_matrix_top(self, msg: OccupancyGrid) -> Optional[np.ndarray]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return None
        data = np.asarray(msg.data, dtype=np.int16)
        if data.size != (width * height):
            return None
        return np.flipud(data.reshape((height, width)))

    def _sample_grid_world(
        self, msg: OccupancyGrid, xs: np.ndarray, ys: np.ndarray
    ) -> Optional[np.ndarray]:
        matrix = self._occupancy_to_matrix_top(msg)
        if matrix is None:
            return None
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            return None
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)

        cols = np.floor((xs - origin_x) / resolution).astype(np.int32)
        rows_bottom = np.floor((ys - origin_y) / resolution).astype(np.int32)
        inside = (
            (cols >= 0)
            & (cols < width)
            & (rows_bottom >= 0)
            & (rows_bottom < height)
        )

        sampled = np.full(xs.shape, -1, dtype=np.int16)
        if np.any(inside):
            rows_top = (height - 1) - rows_bottom
            sampled[inside] = matrix[rows_top[inside], cols[inside]]
        return sampled

    def _cost_values_to_bgr(self, values: np.ndarray) -> np.ndarray:
        clipped = np.clip(values, 0, 100).astype(np.float32)
        gray = np.rint(255.0 - (2.55 * clipped)).astype(np.uint8)
        out = np.stack([gray, gray, gray], axis=-1)
        out[values < 0] = np.array([45, 45, 45], dtype=np.uint8)
        return out

    def _canvas_grid_world_coords(
        self,
        center_x: float,
        center_y: float,
        extent_m: float,
        size_px: int,
    ) -> Tuple[np.ndarray, np.ndarray, float, float]:
        cell_m = extent_m / float(size_px)
        x_min = center_x - (0.5 * extent_m)
        y_max = center_y + (0.5 * extent_m)
        cols = np.arange(size_px, dtype=np.float32)
        rows = np.arange(size_px, dtype=np.float32)
        xs_1d = x_min + ((cols + 0.5) * cell_m)
        ys_1d = y_max - ((rows + 0.5) * cell_m)
        xs, ys = np.meshgrid(xs_1d, ys_1d)
        return xs, ys, x_min, y_max

    def _world_to_canvas_pixels(
        self,
        points_xy: np.ndarray,
        x_min: float,
        y_max: float,
        extent_m: float,
        size_px: int,
    ) -> np.ndarray:
        if points_xy.size == 0:
            return np.empty((0, 2), dtype=np.int32)
        u = np.rint(((points_xy[:, 0] - x_min) / extent_m) * size_px).astype(np.int32)
        v = np.rint(((y_max - points_xy[:, 1]) / extent_m) * size_px).astype(np.int32)
        return np.stack([u, v], axis=1)

    def _polygon_points_in_frame(
        self, polygon_msg: PolygonStamped, target_frame: str
    ) -> Optional[np.ndarray]:
        pts = polygon_msg.polygon.points
        if len(pts) < 3:
            return None
        src_frame = polygon_msg.header.frame_id or target_frame
        points_xy = np.array([[float(p.x), float(p.y)] for p in pts], dtype=np.float32)
        if src_frame == target_frame:
            return points_xy

        tf = self._lookup_transform_2d(target_frame, src_frame, timeout_sec=0.05)
        if tf is None:
            return None
        x_t, y_t = self._apply_transform_2d(
            points_xy[:, 0], points_xy[:, 1], tf[0], tf[1], tf[2], tf[3]
        )
        return np.stack([x_t, y_t], axis=1)

    def _marker_polygons_in_frame(
        self, marker_array: MarkerArray, target_frame: str
    ) -> List[np.ndarray]:
        polygons: List[np.ndarray] = []
        for marker in marker_array.markers:
            if len(marker.points) < 3:
                continue
            src_frame = marker.header.frame_id or target_frame
            points_xy = np.array(
                [[float(p.x), float(p.y)] for p in marker.points], dtype=np.float32
            )
            if src_frame != target_frame:
                tf = self._lookup_transform_2d(target_frame, src_frame, timeout_sec=0.05)
                if tf is None:
                    continue
                x_t, y_t = self._apply_transform_2d(
                    points_xy[:, 0], points_xy[:, 1], tf[0], tf[1], tf[2], tf[3]
                )
                points_xy = np.stack([x_t, y_t], axis=1)
            polygons.append(points_xy)
        return polygons

    def _draw_polygon_on_canvas(
        self,
        canvas: np.ndarray,
        points_xy: np.ndarray,
        x_min: float,
        y_max: float,
        extent_m: float,
        size_px: int,
        color_bgr: Tuple[int, int, int],
        thickness: int = 2,
    ) -> None:
        pix = self._world_to_canvas_pixels(points_xy, x_min, y_max, extent_m, size_px)
        if pix.shape[0] < 3:
            return
        cv2.polylines(
            canvas,
            [pix.reshape((-1, 1, 2))],
            isClosed=True,
            color=color_bgr,
            thickness=thickness,
            lineType=cv2.LINE_AA,
        )

    def _draw_scan_on_canvas(
        self,
        canvas: np.ndarray,
        scan_msg: LaserScan,
        target_frame: str,
        x_min: float,
        y_max: float,
        extent_m: float,
        size_px: int,
    ) -> bool:
        if not scan_msg.ranges:
            return False
        ranges = np.asarray(scan_msg.ranges, dtype=np.float32)
        angles = scan_msg.angle_min + (
            np.arange(ranges.shape[0], dtype=np.float32) * scan_msg.angle_increment
        )
        finite = np.isfinite(ranges)
        finite &= ranges >= float(scan_msg.range_min)
        finite &= ranges <= float(scan_msg.range_max)
        if not np.any(finite):
            return False
        ranges = ranges[finite]
        angles = angles[finite]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        src_frame = scan_msg.header.frame_id or "base_footprint"
        if src_frame != target_frame:
            tf = self._lookup_transform_2d(target_frame, src_frame, timeout_sec=0.05)
            if tf is None:
                return False
            xs, ys = self._apply_transform_2d(xs, ys, tf[0], tf[1], tf[2], tf[3])

        points = np.stack([xs, ys], axis=1)
        pix = self._world_to_canvas_pixels(points, x_min, y_max, extent_m, size_px)
        inside = (
            (pix[:, 0] >= 0)
            & (pix[:, 0] < size_px)
            & (pix[:, 1] >= 0)
            & (pix[:, 1] < size_px)
        )
        if not np.any(inside):
            return False
        pix = pix[inside]
        canvas[pix[:, 1], pix[:, 0]] = np.array([255, 255, 0], dtype=np.uint8)
        return True

    def _path_points_in_frame(
        self, path_msg: NavPath, target_frame: str
    ) -> Optional[np.ndarray]:
        if not path_msg.poses:
            return None
        points_xy = np.array(
            [
                [float(pose.pose.position.x), float(pose.pose.position.y)]
                for pose in path_msg.poses
            ],
            dtype=np.float32,
        )
        if points_xy.shape[0] < 2:
            return None

        src_frame = path_msg.header.frame_id or target_frame
        if src_frame == target_frame:
            return points_xy

        tf = self._lookup_transform_2d(target_frame, src_frame, timeout_sec=0.05)
        if tf is None:
            return None
        x_t, y_t = self._apply_transform_2d(
            points_xy[:, 0], points_xy[:, 1], tf[0], tf[1], tf[2], tf[3]
        )
        return np.stack([x_t, y_t], axis=1)

    def _draw_path_on_canvas(
        self,
        canvas: np.ndarray,
        points_xy: np.ndarray,
        x_min: float,
        y_max: float,
        extent_m: float,
        size_px: int,
        color_bgr: Tuple[int, int, int],
        thickness: int = 2,
    ) -> bool:
        pix = self._world_to_canvas_pixels(points_xy, x_min, y_max, extent_m, size_px)
        if pix.shape[0] < 2:
            return False
        cv2.polylines(
            canvas,
            [pix.reshape((-1, 1, 2))],
            isClosed=False,
            color=color_bgr,
            thickness=thickness,
            lineType=cv2.LINE_AA,
        )
        return True

    def _draw_global_inset(
        self,
        canvas: np.ndarray,
        global_costmap: OccupancyGrid,
        robot_map_pose: Optional[Tuple[float, float, float]],
    ) -> bool:
        matrix = self._occupancy_to_matrix_top(global_costmap)
        if matrix is None:
            return False
        inset_px = min(self.snapshot_global_inset_px, max(64, canvas.shape[0] // 2))
        inset = self._cost_values_to_bgr(matrix)
        inset = cv2.resize(inset, (inset_px, inset_px), interpolation=cv2.INTER_NEAREST)

        if robot_map_pose is not None:
            x_map = float(robot_map_pose[0])
            y_map = float(robot_map_pose[1])
            width = int(global_costmap.info.width)
            height = int(global_costmap.info.height)
            resolution = float(global_costmap.info.resolution)
            origin_x = float(global_costmap.info.origin.position.x)
            origin_y = float(global_costmap.info.origin.position.y)
            if resolution > 0.0 and width > 0 and height > 0:
                col = int(math.floor((x_map - origin_x) / resolution))
                row_bottom = int(math.floor((y_map - origin_y) / resolution))
                if (0 <= col < width) and (0 <= row_bottom < height):
                    row_top = (height - 1) - row_bottom
                    u = int(round((float(col) / max(1.0, float(width - 1))) * (inset_px - 1)))
                    v = int(
                        round((float(row_top) / max(1.0, float(height - 1))) * (inset_px - 1))
                    )
                    cv2.circle(inset, (u, v), 4, (0, 0, 255), thickness=-1)

        margin = 10
        x0 = max(0, canvas.shape[1] - inset_px - margin)
        y0 = margin
        canvas[y0 : y0 + inset_px, x0 : x0 + inset_px] = inset
        cv2.rectangle(
            canvas,
            (x0 - 1, y0 - 1),
            (x0 + inset_px, y0 + inset_px),
            (255, 255, 255),
            1,
        )
        return True

    def build_nav_snapshot_payload(self) -> Dict[str, Any]:
        timeout_sec = max(0.05, float(self.snapshot_timeout_ms) / 1000.0)
        deadline = time.monotonic() + timeout_sec
        local_costmap: Optional[OccupancyGrid] = None
        global_costmap: Optional[OccupancyGrid] = None
        keepout_mask: Optional[OccupancyGrid] = None
        local_footprint: Optional[PolygonStamped] = None
        stop_zone: Optional[PolygonStamped] = None
        collision_polygons: Optional[MarkerArray] = None
        scan_msg: Optional[LaserScan] = None
        plan_msg: Optional[NavPath] = None

        while time.monotonic() <= deadline:
            with self._lock:
                local_costmap = self._last_local_costmap
                global_costmap = self._last_global_costmap
                keepout_mask = self._last_keepout_mask
                local_footprint = self._last_local_footprint
                stop_zone = self._last_stop_zone
                collision_polygons = self._last_collision_polygons
                scan_msg = self._last_scan
                plan_msg = self._last_plan
            if local_costmap is not None and global_costmap is not None:
                break
            time.sleep(0.02)

        if local_costmap is None or global_costmap is None:
            missing = []
            if local_costmap is None:
                missing.append(self.local_costmap_topic)
            if global_costmap is None:
                missing.append(self.global_costmap_topic)
            return {
                "op": "nav_snapshot",
                "ok": False,
                "error": f"Missing required topics: {', '.join(missing)}",
            }

        local_frame = local_costmap.header.frame_id or "odom"
        robot_local = self._lookup_robot_pose_in_frame(local_frame, timeout_sec=0.1)
        if robot_local is None:
            return {
                "op": "nav_snapshot",
                "ok": False,
                "error": f"TF unavailable for base_footprint in frame '{local_frame}'",
            }

        size_px = int(self.snapshot_size_px)
        extent_m = float(self.snapshot_extent_m)
        center_x = float(robot_local[0])
        center_y = float(robot_local[1])

        xs_local, ys_local, x_min, y_max = self._canvas_grid_world_coords(
            center_x, center_y, extent_m, size_px
        )
        local_values = self._sample_grid_world(local_costmap, xs_local, ys_local)
        if local_values is None:
            return {
                "op": "nav_snapshot",
                "ok": False,
                "error": "Local costmap payload invalid",
            }
        canvas = self._cost_values_to_bgr(local_values)

        layers: Dict[str, bool] = {
            "local_costmap": True,
            "global_costmap": False,
            "keepout_mask": False,
            "local_footprint": False,
            "stop_zone": False,
            "collision_polygons": False,
            "scan": False,
            "plan": False,
        }

        if keepout_mask is not None:
            keepout_frame = keepout_mask.header.frame_id or self.map_frame
            if keepout_frame == local_frame:
                xs_k, ys_k = xs_local, ys_local
            else:
                tf = self._lookup_transform_2d(keepout_frame, local_frame, timeout_sec=0.05)
                if tf is not None:
                    xs_k, ys_k = self._apply_transform_2d(
                        xs_local, ys_local, tf[0], tf[1], tf[2], tf[3]
                    )
                else:
                    xs_k = ys_k = None
            if xs_k is not None and ys_k is not None:
                keepout_values = self._sample_grid_world(keepout_mask, xs_k, ys_k)
                if keepout_values is not None:
                    band = keepout_values > 0
                    if np.any(band):
                        alpha = np.clip(
                            keepout_values.astype(np.float32) / 100.0, 0.1, 0.65
                        )
                        overlay = np.zeros_like(canvas, dtype=np.float32)
                        overlay[:, :, 0] = 40.0
                        overlay[:, :, 1] = 140.0
                        overlay[:, :, 2] = 255.0
                        canvas_f = canvas.astype(np.float32)
                        a = alpha[:, :, None]
                        canvas_f[band] = (
                            (1.0 - a[band]) * canvas_f[band]
                            + a[band] * overlay[band]
                        )
                        canvas = np.clip(canvas_f, 0, 255).astype(np.uint8)
                        layers["keepout_mask"] = True

        if local_footprint is not None:
            footprint_xy = self._polygon_points_in_frame(local_footprint, local_frame)
            if footprint_xy is not None:
                self._draw_polygon_on_canvas(
                    canvas,
                    footprint_xy,
                    x_min,
                    y_max,
                    extent_m,
                    size_px,
                    color_bgr=(0, 255, 0),
                )
                layers["local_footprint"] = True

        if stop_zone is not None:
            stop_xy = self._polygon_points_in_frame(stop_zone, local_frame)
            if stop_xy is not None:
                self._draw_polygon_on_canvas(
                    canvas,
                    stop_xy,
                    x_min,
                    y_max,
                    extent_m,
                    size_px,
                    color_bgr=(0, 0, 255),
                )
                layers["stop_zone"] = True

        if collision_polygons is not None:
            marker_polys = self._marker_polygons_in_frame(collision_polygons, local_frame)
            if marker_polys:
                for poly in marker_polys:
                    self._draw_polygon_on_canvas(
                        canvas,
                        poly,
                        x_min,
                        y_max,
                        extent_m,
                        size_px,
                        color_bgr=(255, 0, 255),
                        thickness=1,
                    )
                layers["collision_polygons"] = True

        if scan_msg is not None:
            layers["scan"] = self._draw_scan_on_canvas(
                canvas, scan_msg, local_frame, x_min, y_max, extent_m, size_px
            )
        if plan_msg is not None:
            plan_xy = self._path_points_in_frame(plan_msg, local_frame)
            if plan_xy is not None:
                layers["plan"] = self._draw_path_on_canvas(
                    canvas,
                    plan_xy,
                    x_min,
                    y_max,
                    extent_m,
                    size_px,
                    color_bgr=(0, 165, 255),
                    thickness=2,
                )

        robot_map_pose = self._lookup_robot_pose_in_frame(
            global_costmap.header.frame_id or self.map_frame, timeout_sec=0.05
        )
        layers["global_costmap"] = self._draw_global_inset(
            canvas, global_costmap, robot_map_pose
        )
        if not layers["global_costmap"]:
            return {
                "op": "nav_snapshot",
                "ok": False,
                "error": "Global costmap payload invalid",
            }

        cv2.circle(
            canvas,
            (size_px // 2, size_px // 2),
            4,
            color=(0, 0, 255),
            thickness=-1,
        )

        encoded_ok, encoded = cv2.imencode(".png", canvas)
        if not encoded_ok:
            return {
                "op": "nav_snapshot",
                "ok": False,
                "error": "Failed encoding PNG snapshot",
            }

        image_bytes = encoded.tobytes()
        image_b64 = base64.b64encode(image_bytes).decode("ascii")
        stamp = local_costmap.header.stamp
        return {
            "op": "nav_snapshot",
            "ok": True,
            "mime": "image/png",
            "width": int(canvas.shape[1]),
            "height": int(canvas.shape[0]),
            "frame_id": local_frame,
            "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
            "layers": layers,
            "image_b64": image_b64,
            "image_size_bytes": int(len(image_bytes)),
        }

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

        with self._lock:
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

        with self._lock:
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

    def _resolve_mask_grid_spec(self) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        if self.use_fixed_mask_grid:
            if self._fixed_mask_info is None:
                self._fixed_mask_info = self._build_fixed_mask_metadata()
            info = copy.deepcopy(self._fixed_mask_info)
            return (
                {
                    "source": "fixed_mask_grid",
                    "frame_id": self.map_frame,
                    "width": int(info.width),
                    "height": int(info.height),
                    "resolution": float(info.resolution),
                    "origin": info.origin,
                },
                None,
            )

        map_msg = self._get_global_costmap()
        if map_msg is None:
            return None, f"Service {self.global_costmap_service} failed"
        md = map_msg.metadata
        return (
            {
                "source": "global_costmap",
                "frame_id": map_msg.header.frame_id or self.map_frame,
                "width": int(md.size_x),
                "height": int(md.size_y),
                "resolution": float(md.resolution),
                "origin": md.origin,
            },
            None,
        )

    def _rasterize_mask(
        self, zones_xy: List[Dict[str, Any]], grid_spec: Dict[str, Any]
    ) -> OccupancyGrid:
        width = int(grid_spec["width"])
        height = int(grid_spec["height"])
        resolution = float(grid_spec["resolution"])
        origin = grid_spec["origin"]
        origin_x = float(origin.position.x)
        origin_y = float(origin.position.y)

        core_mask, clipped_vertices, outside_zone_ids = rasterize_polygons_core(
            zones_xy=zones_xy,
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        for zone_id, clipped_count in clipped_vertices.items():
            self.get_logger().warning(
                f"Zone '{zone_id}' clipped {clipped_count} vertices to mask bounds"
            )
        for zone_id in outside_zone_ids:
            self.get_logger().warning(
                f"Zone '{zone_id}' is completely outside mask bounds and was skipped"
            )

        if self.degrade_enabled and self.degrade_radius_m > 0.0:
            gradient_mask = exponential_gradient_from_core(
                core_mask=core_mask,
                resolution=resolution,
                radius_m=self.degrade_radius_m,
                edge_cost=self.degrade_edge_cost,
                min_cost=self.degrade_min_cost,
                use_l2=self.degrade_use_l2,
            )
            mask = np.maximum(gradient_mask, core_mask)
            mask[core_mask > 0] = 100
        else:
            mask = core_mask

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(grid_spec["frame_id"])
        out.info.map_load_time = out.header.stamp
        out.info.resolution = resolution
        out.info.width = width
        out.info.height = height
        out.info.origin = origin
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

        grid_spec, err = self._resolve_mask_grid_spec()
        if grid_spec is None:
            return False, str(err), False

        zones_xy = self._convert_zones_to_xy(zones)
        started = time.perf_counter()
        occ = self._rasterize_mask(zones_xy, grid_spec)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.get_logger().info(
            f"Mask rasterized in {elapsed_ms:.1f} ms "
            f"(zones={len(zones_xy)}, source={grid_spec['source']}, "
            f"degrade={'on' if self.degrade_enabled and self.degrade_radius_m > 0.0 else 'off'})"
        )

        self._mask_pub.publish(occ)

        with self._lock:
            self._zones_ll = zones
            self._zones_xy = zones_xy
            self._mask_ready = True
            self._mask_source = str(grid_spec["source"])

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

        if op == "get_nav_snapshot":
            started = time.perf_counter()
            payload = await asyncio.to_thread(self.node.build_nav_snapshot_payload)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if payload.get("ok") is True:
                self.node.get_logger().info(
                    f"Nav snapshot generated in {elapsed_ms:.1f} ms "
                    f"(png={payload.get('image_size_bytes', 0)} bytes)"
                )
            else:
                self.node.get_logger().warning(
                    f"Nav snapshot failed in {elapsed_ms:.1f} ms: {payload.get('error')}"
                )
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
