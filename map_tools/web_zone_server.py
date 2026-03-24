import asyncio
import base64
from collections import deque
import json
import math
import os
import shlex
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
import websockets
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry
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
from .waypoints_file_utils import load_waypoints_yaml_file, save_waypoints_yaml_file


ROSBAG_TOPIC_PROFILES: Dict[str, Tuple[str, ...]] = {
    "core": (
        "/gps/fix",
        "/odometry/local",
        "/odometry/gps",
        "/imu/data",
        "/scan",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/cmd_vel_final",
        "/collision_monitor_state",
        "/nav_command_server/telemetry",
        "/nav_command_server/events",
        "/controller/status",
        "/controller/telemetry",
        "/diagnostics",
        "/tf",
        "/tf_static",
        "/rosout",
    ),
    "full_nav2": (
        "/gps/fix",
        "/odometry/local",
        "/odometry/gps",
        "/imu/data",
        "/scan",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/cmd_vel_final",
        "/collision_monitor_state",
        "/nav_command_server/telemetry",
        "/nav_command_server/events",
        "/controller/status",
        "/controller/telemetry",
        "/diagnostics",
        "/tf",
        "/tf_static",
        "/rosout",
        "/plan",
        "/local_costmap/costmap",
        "/global_costmap/costmap",
        "/local_costmap/published_footprint",
        "/behavior_tree_log",
    ),
}

UNSET = object()


class WebZoneServerNode(Node):
    @staticmethod
    def _diag_level_value(value: Any) -> int:
        if isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder="little", signed=False)
        return int(value)

    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("web_zone_server")
        self._loop = loop

        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8766)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("odom_topic", "/odometry/local")
        self.declare_parameter("gps_broadcast_hz", 1.0)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("snapshot_request_timeout_s", 2.0)
        self.declare_parameter("set_zones_timeout_s", 12.0)
        self.declare_parameter("set_goal_timeout_s", 12.0)
        self.declare_parameter("waypoints_file", "")

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
        self.declare_parameter("nav_events_topic", "/nav_command_server/events")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("rosbag_output_dir", "/ros2_ws/bags")
        self.declare_parameter("camera_pan_service", "/camara/camera_pan")
        self.declare_parameter("camera_zoom_toggle_service", "/camara/camera_zoom_toggle")
        self.declare_parameter("camera_status_service", "/camara/camera_status")

        self.ws_host = str(self.get_parameter("ws_host").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
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
        configured_waypoints_file = str(self.get_parameter("waypoints_file").value)
        self.waypoints_file = self._resolve_waypoints_file(configured_waypoints_file)

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
        self.nav_events_topic = str(self.get_parameter("nav_events_topic").value)
        self.diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        self.rosbag_output_dir = str(self.get_parameter("rosbag_output_dir").value)
        self.camera_pan_service = str(self.get_parameter("camera_pan_service").value)
        self.camera_zoom_toggle_service = str(
            self.get_parameter("camera_zoom_toggle_service").value
        )
        self.camera_status_service = str(self.get_parameter("camera_status_service").value)

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()
        self._ws_send_locks: Dict[Any, asyncio.Lock] = {}

        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_robot_heading_deg: Optional[float] = None
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
        self._nav_result_status = 0
        self._nav_result_text = "idle"
        self._nav_result_event_id = 0
        self._camera_status = {
            "ok": False,
            "error": "camera status unavailable",
            "last_command": "none",
            "zoom_in": False,
        }
        self._recent_nav_events: deque[Dict[str, Any]] = deque(maxlen=30)
        self._active_alerts: List[Dict[str, Any]] = []
        self._rosbag_process: Optional[subprocess.Popen] = None
        self._rosbag_profile = ""
        self._rosbag_output_path = ""
        self._rosbag_log_path = ""
        self._rosbag_started_at_epoch_ms: Optional[int] = None
        self._rosbag_last_exit_code: Optional[int] = None
        self._rosbag_last_error = ""

        self._manual_cmd_last_monotonic: Optional[float] = None

        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._on_gps_fix, qos_profile_sensor_data
        )
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odometry, 10
        )
        self._nav_telemetry_sub = self.create_subscription(
            NavTelemetry, self.nav_telemetry_topic, self._on_nav_telemetry, 10
        )
        self._nav_events_sub = self.create_subscription(
            NavEvent, self.nav_events_topic, self._on_nav_event, 10
        )
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray, self.diagnostics_topic, self._on_diagnostics, 10
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
        self._teleop_cmd_pub = self.create_publisher(CmdVelFinal, self.teleop_cmd_topic, 10)
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
            f"nav_events={self.nav_events_topic}, diagnostics={self.diagnostics_topic}, "
            f"rosbag_dir={self.rosbag_output_dir}, "
            f"camera_pan={self.camera_pan_service}, camera_zoom_toggle={self.camera_zoom_toggle_service}, "
            f"camera_status={self.camera_status_service}, "
            f"teleop_topic={self.teleop_cmd_topic}, gps_topic={self.gps_topic}, "
            f"odom_topic={self.odom_topic})"
        )
        self.get_logger().info(f"Waypoints file path: {self.waypoints_file}")

    def add_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.add(ws)
            self._ws_send_locks[ws] = asyncio.Lock()
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client connected (clients={count})")

    def remove_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.discard(ws)
            self._ws_send_locks.pop(ws, None)
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client disconnected (clients={count})")

    async def send_ws_text(self, ws: Any, text: str) -> bool:
        with self._lock:
            lock = self._ws_send_locks.get(ws)
        if lock is None:
            return False
        async with lock:
            await ws.send(text)
        return True

    async def send_ws_json(self, ws: Any, payload: Dict[str, Any]) -> bool:
        return await self.send_ws_text(ws, json.dumps(payload))

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
                "nav_result_status": int(self._nav_result_status),
                "nav_result_text": str(self._nav_result_text),
                "nav_result_event_id": int(self._nav_result_event_id),
                "alerts": list(self._active_alerts),
                "recent_events": list(self._recent_nav_events),
                "rosbag": self._build_rosbag_status_payload_locked(),
                "camera_status": dict(self._camera_status),
            }

    def _build_nav_telemetry_payload(self) -> Dict[str, Any]:
        with self._lock:
            cmd_vel_safe = dict(self._cmd_vel_safe)
            manual_control = dict(self._manual_control)
            goal_active = bool(self._goal_active)
            nav_result_status = int(self._nav_result_status)
            nav_result_text = str(self._nav_result_text)
            nav_result_event_id = int(self._nav_result_event_id)
            alerts = list(self._active_alerts)
            recent_events = list(self._recent_nav_events)
        return {
            "op": "nav_telemetry",
            "cmd_vel_safe": cmd_vel_safe,
            "manual_control": manual_control,
            "goal_active": goal_active,
            "nav_result_status": nav_result_status,
            "nav_result_text": nav_result_text,
            "nav_result_event_id": nav_result_event_id,
            "alerts": alerts,
            "recent_events": recent_events,
        }

    @staticmethod
    def _rosbag_topics_for_profile(profile: str) -> Optional[Tuple[str, ...]]:
        return ROSBAG_TOPIC_PROFILES.get(str(profile))

    def _build_rosbag_status_payload_locked(self) -> Dict[str, Any]:
        active = self._rosbag_process is not None and self._rosbag_process.poll() is None
        pid = None
        if active and self._rosbag_process is not None:
            pid = int(self._rosbag_process.pid)
        return {
            "active": bool(active),
            "profile": str(self._rosbag_profile),
            "output_dir": str(self._rosbag_output_path),
            "log_path": str(self._rosbag_log_path),
            "pid": pid,
            "started_at_epoch_ms": (
                int(self._rosbag_started_at_epoch_ms)
                if self._rosbag_started_at_epoch_ms is not None
                else None
            ),
            "last_exit_code": (
                int(self._rosbag_last_exit_code)
                if self._rosbag_last_exit_code is not None
                else None
            ),
            "last_error": str(self._rosbag_last_error),
            "available_profiles": sorted(ROSBAG_TOPIC_PROFILES.keys()),
        }

    def _rosbag_status_payload(self) -> Dict[str, Any]:
        with self._lock:
            return self._build_rosbag_status_payload_locked()

    @staticmethod
    def _nav_event_details_to_dict(msg: NavEvent) -> Dict[str, str]:
        details: Dict[str, str] = {}
        for item in getattr(msg, "details", []) or []:
            key = str(getattr(item, "key", "") or "")
            if not key:
                continue
            details[key] = str(getattr(item, "value", "") or "")
        return details

    @staticmethod
    def _diagnostic_values_to_dict(status: Any) -> Dict[str, str]:
        values: Dict[str, str] = {}
        for item in getattr(status, "values", []) or []:
            key = str(getattr(item, "key", "") or "")
            if not key:
                continue
            values[key] = str(getattr(item, "value", "") or "")
        return values

    def _nav_event_to_payload(self, msg: NavEvent) -> Dict[str, Any]:
        return {
            "stamp": {
                "sec": int(getattr(msg.stamp, "sec", 0)),
                "nanosec": int(getattr(msg.stamp, "nanosec", 0)),
            },
            "severity": int(msg.severity),
            "component": str(msg.component),
            "code": str(msg.code),
            "message": str(msg.message),
            "event_id": int(msg.event_id),
            "details": self._nav_event_details_to_dict(msg),
        }

    def _diagnostic_status_to_payload(self, status: Any) -> Dict[str, Any]:
        return {
            "name": str(getattr(status, "name", "")),
            "level": self._diag_level_value(
                getattr(
                    status,
                    "level",
                    self._diag_level_value(DiagnosticStatus.OK),
                )
            ),
            "message": str(getattr(status, "message", "")),
            "hardware_id": str(getattr(status, "hardware_id", "")),
            "values": self._diagnostic_values_to_dict(status),
        }

    def _should_surface_diagnostic(self, status: Any) -> bool:
        level = self._diag_level_value(
            getattr(
                status,
                "level",
                self._diag_level_value(DiagnosticStatus.OK),
            )
        )
        if level == self._diag_level_value(DiagnosticStatus.OK):
            return False
        name = str(getattr(status, "name", "") or "")
        if not name.startswith("navigation/"):
            return False
        message = str(getattr(status, "message", "") or "")
        if name == "navigation/collision_monitor" and message == "no collision monitor state yet":
            return False
        return True

    def _broadcast_from_thread(self, payload: Dict[str, Any]) -> None:
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _broadcast_rosbag_status(self) -> None:
        self._broadcast_from_thread(
            {
                "op": "rosbag_status",
                "rosbag": self._rosbag_status_payload(),
            }
        )

    def _update_rosbag_state_locked(
        self,
        *,
        process: Any = UNSET,
        profile: Any = UNSET,
        output_path: Any = UNSET,
        log_path: Any = UNSET,
        started_at_epoch_ms: Any = UNSET,
        last_exit_code: Any = UNSET,
        last_error: Any = UNSET,
    ) -> None:
        if process is not UNSET:
            self._rosbag_process = process
        if profile is not UNSET:
            self._rosbag_profile = str(profile)
        if output_path is not UNSET:
            self._rosbag_output_path = str(output_path)
        if log_path is not UNSET:
            self._rosbag_log_path = str(log_path)
        if started_at_epoch_ms is not UNSET:
            if started_at_epoch_ms is None:
                self._rosbag_started_at_epoch_ms = None
            else:
                self._rosbag_started_at_epoch_ms = int(started_at_epoch_ms)
        if last_exit_code is not UNSET:
            if last_exit_code is None:
                self._rosbag_last_exit_code = None
            else:
                self._rosbag_last_exit_code = int(last_exit_code)
        if last_error is not UNSET:
            self._rosbag_last_error = str(last_error)

    def _rosbag_waiter(self, process: subprocess.Popen) -> None:
        exit_code = process.wait()
        with self._lock:
            if self._rosbag_process is not process:
                return
            self._rosbag_process = None
            self._rosbag_last_exit_code = int(exit_code)
            if exit_code == 0:
                self._rosbag_last_error = ""
            elif not self._rosbag_last_error:
                self._rosbag_last_error = f"rosbag exited with code {exit_code}"
        self._broadcast_rosbag_status()

    def get_rosbag_status(self) -> Dict[str, Any]:
        return self._rosbag_status_payload()

    def start_rosbag(self, profile: str = "core") -> Tuple[bool, str, Dict[str, Any]]:
        profile_name = str(profile or "core").strip() or "core"
        topics = self._rosbag_topics_for_profile(profile_name)
        if topics is None:
            return False, f"unknown rosbag profile: {profile_name}", self.get_rosbag_status()

        with self._lock:
            if self._rosbag_process is not None and self._rosbag_process.poll() is None:
                return False, "rosbag is already running", self._build_rosbag_status_payload_locked()

        bags_dir = Path(self.rosbag_output_dir)
        bags_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        output_dir = bags_dir / f"nav_debug_{profile_name}_{stamp}"
        log_path = bags_dir / f"nav_debug_{profile_name}_{stamp}.log"

        output_dir_quoted = shlex.quote(str(output_dir))
        topics_quoted = " ".join(shlex.quote(topic) for topic in topics)
        cmd = (
            "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && "
            "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi && "
            "cd /ros2_ws && "
            f"exec ros2 bag record -o {output_dir_quoted} {topics_quoted}"
        )

        with log_path.open("ab") as log_file:
            process = subprocess.Popen(
                ["bash", "-lc", cmd],
                cwd="/ros2_ws",
                stdout=log_file,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

        time.sleep(0.4)
        exit_code = process.poll()
        if exit_code is not None:
            err = f"rosbag failed to start (exit_code={exit_code})"
            with self._lock:
                self._update_rosbag_state_locked(
                    process=None,
                    profile=profile_name,
                    output_path=str(output_dir),
                    log_path=str(log_path),
                    started_at_epoch_ms=None,
                    last_exit_code=int(exit_code),
                    last_error=err,
                )
            self._broadcast_rosbag_status()
            return False, err, self.get_rosbag_status()

        started_at_epoch_ms = int(time.time() * 1000.0)
        with self._lock:
            self._update_rosbag_state_locked(
                process=process,
                profile=profile_name,
                output_path=str(output_dir),
                log_path=str(log_path),
                started_at_epoch_ms=started_at_epoch_ms,
                last_exit_code=None,
                last_error="",
            )
        waiter = threading.Thread(
            target=self._rosbag_waiter,
            args=(process,),
            daemon=True,
            name="rosbag_waiter",
        )
        waiter.start()
        self._broadcast_rosbag_status()
        return True, "", self.get_rosbag_status()

    def stop_rosbag(self) -> Tuple[bool, str, Dict[str, Any]]:
        with self._lock:
            process = self._rosbag_process
        if process is None or process.poll() is not None:
            with self._lock:
                self._rosbag_process = None
            return False, "rosbag is not running", self.get_rosbag_status()

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except Exception:
            try:
                process.send_signal(signal.SIGINT)
            except Exception as exc:
                return False, f"failed to stop rosbag: {exc}", self.get_rosbag_status()

        deadline = time.time() + 10.0
        while time.time() < deadline:
            if process.poll() is not None:
                break
            time.sleep(0.1)
        if process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    process.kill()
                process.wait(timeout=5.0)

        with self._lock:
            if self._rosbag_process is process:
                self._rosbag_process = None
                self._rosbag_last_exit_code = int(process.returncode or 0)
                if int(process.returncode or 0) == 0:
                    self._rosbag_last_error = ""
        self._broadcast_rosbag_status()
        return True, "", self.get_rosbag_status()

    def close(self) -> None:
        try:
            self.stop_rosbag()
        except Exception:
            pass

    async def _broadcast(self, payload: Dict[str, Any]) -> None:
        text = json.dumps(payload)
        with self._lock:
            clients = list(self._ws_clients)
        if not clients:
            return
        failed = []
        for ws in clients:
            try:
                sent = await self.send_ws_text(ws, text)
                if not sent:
                    failed.append(ws)
            except Exception:
                failed.append(ws)
        if failed:
            with self._lock:
                for ws in failed:
                    self._ws_clients.discard(ws)
                    self._ws_send_locks.pop(ws, None)

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not np.isfinite(msg.latitude) or not np.isfinite(msg.longitude):
            return

        with self._lock:
            heading_deg = self._last_robot_heading_deg
        pose = self._build_robot_pose(
            lat=float(msg.latitude),
            lon=float(msg.longitude),
            heading_deg=heading_deg,
        )
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

    def _yaw_deg_from_quaternion(
        self, x: float, y: float, z: float, w: float
    ) -> Optional[float]:
        if (
            (not np.isfinite(x))
            or (not np.isfinite(y))
            or (not np.isfinite(z))
            or (not np.isfinite(w))
        ):
            return None
        norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
        if norm < 1.0e-9:
            return None
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        while yaw_deg <= -180.0:
            yaw_deg += 360.0
        while yaw_deg > 180.0:
            yaw_deg -= 360.0
        return float(yaw_deg)

    def _build_robot_pose(
        self, lat: float, lon: float, heading_deg: Optional[float] = None
    ) -> Dict[str, float]:
        pose = {"lat": float(lat), "lon": float(lon)}
        if heading_deg is not None and np.isfinite(heading_deg):
            pose["heading_deg"] = float(heading_deg)
        return pose

    def _on_odometry(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        heading_deg = self._yaw_deg_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        if heading_deg is None:
            return
        with self._lock:
            self._last_robot_heading_deg = float(heading_deg)
            if self._last_robot_pose is not None:
                self._last_robot_pose["heading_deg"] = float(heading_deg)

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        robot_pose_payload = None
        with self._lock:
            self._cmd_vel_safe = {
                "available": bool(msg.cmd_vel_available),
                "linear_x": float(msg.cmd_vel_linear_x),
                "angular_z": float(msg.cmd_vel_angular_z),
            }
            self._goal_active = bool(msg.goal_active)
            self._nav_result_status = int(getattr(msg, "nav_result_status", 0))
            self._nav_result_text = str(getattr(msg, "nav_result_text", ""))
            self._nav_result_event_id = int(getattr(msg, "nav_result_event_id", 0))

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
                self._last_robot_pose = self._build_robot_pose(
                    lat=float(msg.robot_lat),
                    lon=float(msg.robot_lon),
                    heading_deg=self._last_robot_heading_deg,
                )
                robot_pose_payload = dict(self._last_robot_pose)

        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )
        if robot_pose_payload is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"op": "robot_pose", "pose": robot_pose_payload}),
                self._loop,
            )

    def _on_nav_event(self, msg: NavEvent) -> None:
        payload = self._nav_event_to_payload(msg)
        with self._lock:
            self._recent_nav_events.append(payload)
        asyncio.run_coroutine_threadsafe(
            self._broadcast({"op": "nav_event", "event": payload}), self._loop
        )
        asyncio.run_coroutine_threadsafe(
            self._broadcast(self._build_nav_telemetry_payload()), self._loop
        )

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        alerts = [
            self._diagnostic_status_to_payload(status)
            for status in (msg.status or [])
            if self._should_surface_diagnostic(status)
        ]
        alerts.sort(key=lambda item: (-int(item.get("level", 0)), str(item.get("name", ""))))
        with self._lock:
            self._active_alerts = alerts
        asyncio.run_coroutine_threadsafe(
            self._broadcast({"op": "nav_alerts", "alerts": alerts}), self._loop
        )
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

    def _resolve_waypoints_file(self, configured_path: str) -> Path:
        if configured_path:
            return Path(configured_path)

        config_dir = self._resolve_navegacion_config_dir()
        return config_dir / "saved_waypoints.yaml"

    def _resolve_navegacion_config_dir(self) -> Path:
        try:
            pkg_dir = Path(get_package_share_directory("navegacion_gps"))
            default_dir = pkg_dir / "config"
            try:
                workspace_root = pkg_dir.parents[3]
                source_dir = workspace_root / "src" / "navegacion_gps" / "config"
                if source_dir.exists():
                    return source_dir
            except Exception:
                pass
            return default_dir
        except Exception:
            pass

        fallback = Path(__file__).resolve().parents[3] / "src" / "navegacion_gps" / "config"
        return fallback

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
                self._last_robot_pose = self._build_robot_pose(
                    lat=float(response.robot_lat),
                    lon=float(response.robot_lon),
                    heading_deg=self._last_robot_heading_deg,
                )

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

    def save_waypoints_file(self, waypoints: List[Dict[str, float]]) -> Tuple[bool, str, int]:
        ok, err, count = save_waypoints_yaml_file(self.waypoints_file, waypoints)
        if not ok:
            self.get_logger().warning(f"save_waypoints_file failed: {err}")
            return False, err, 0
        self.get_logger().info(f"save_waypoints_file ok (count={count})")
        return True, "", int(count)

    def load_waypoints_file(self) -> Tuple[bool, str, List[Dict[str, float]]]:
        ok, err, waypoints = load_waypoints_yaml_file(self.waypoints_file)
        if not ok:
            self.get_logger().warning(f"load_waypoints_file failed: {err}")
            return False, err, []
        self.get_logger().info(f"load_waypoints_file ok (count={len(waypoints)})")
        return True, "", waypoints

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

    def set_manual_cmd(
        self, linear_x: float, angular_z: float, brake_pct: int = 0
    ) -> Tuple[bool, str]:
        if not np.isfinite(linear_x) or not np.isfinite(angular_z):
            return False, "invalid manual command values"

        brake_pct_clamped = max(0, min(100, int(brake_pct)))

        cmd = CmdVelFinal()
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        cmd.brake_pct = brake_pct_clamped
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

    async def _reload_zones_on_connect(self) -> None:
        try:
            ok, err = await asyncio.to_thread(self.node.reload_zones_from_disk)
            if not ok:
                if err:
                    self.node.get_logger().warning(
                        f"zones reload on WS connect failed: {err}"
                    )
                return
            await self.node._broadcast(self.node.snapshot_state())
        except Exception as exc:
            self.node.get_logger().warning(f"zones reload on WS connect crashed: {exc}")

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

    @staticmethod
    def _extract_client_req_id(msg: Dict[str, Any]) -> Optional[str]:
        req_id = msg.get("client_req_id")
        if req_id is None:
            return None
        if isinstance(req_id, (str, int, float, bool)):
            return str(req_id)
        return None

    def _build_ack_payload(
        self,
        request: str,
        ok: bool,
        error: Optional[str],
        client_req_id: Optional[str],
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "op": "ack",
            "ok": bool(ok),
            "request": str(request),
            "error": None if ok else str(error or "unknown error"),
        }
        if client_req_id is not None:
            payload["client_req_id"] = client_req_id
        if extra:
            payload.update(extra)
        return payload

    async def _send_json(self, ws: Any, payload: Dict[str, Any]) -> None:
        await self.node.send_ws_json(ws, payload)

    async def _send_ack(
        self,
        ws: Any,
        request: str,
        ok: bool,
        error: Optional[str] = None,
        *,
        client_req_id: Optional[str] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        await self._send_json(
            ws,
            self._build_ack_payload(
                request=request,
                ok=ok,
                error=error,
                client_req_id=client_req_id,
                extra=extra,
            ),
        )

    async def handle(self, ws: Any, path: Optional[str] = None) -> None:
        _ = path
        pending_tasks: Set[asyncio.Task[Any]] = set()
        self.node.add_client(ws)
        try:
            await self._send_json(ws, self.node.snapshot_state())
            connect_reload_task = asyncio.create_task(self._reload_zones_on_connect())
            pending_tasks.add(connect_reload_task)
            connect_reload_task.add_done_callback(
                lambda done: pending_tasks.discard(done)
            )
            async for raw in ws:
                task = asyncio.create_task(self._handle_message_safe(ws, raw))
                pending_tasks.add(task)
                task.add_done_callback(lambda done: pending_tasks.discard(done))
        finally:
            for task in list(pending_tasks):
                task.cancel()
            if pending_tasks:
                await asyncio.gather(*pending_tasks, return_exceptions=True)
            self.node.remove_client(ws)

    async def _handle_message_safe(self, ws: Any, raw: str) -> None:
        try:
            await self._handle_message(ws, raw)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.node.get_logger().error(f"WS request handling failed: {exc}")

    async def _handle_message(self, ws: Any, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            self.node.get_logger().warning("Invalid WS JSON payload received")
            await self._send_ack(ws, "invalid_json", False, "invalid json")
            return

        client_req_id = self._extract_client_req_id(msg)
        op = msg.get("op")
        if op != "set_manual_cmd":
            self.node.get_logger().info(f"WS op received: {op}")
        if op == "get_state":
            payload = self.node.snapshot_state()
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "get_rosbag_status":
            payload = {
                "op": "rosbag_status",
                "rosbag": await asyncio.to_thread(self.node.get_rosbag_status),
            }
            if client_req_id is not None:
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        if op == "set_zones_geojson":
            geojson_payload = msg.get("geojson")
            if geojson_payload is None:
                await self._send_ack(
                    ws,
                    "set_zones_geojson",
                    False,
                    "geojson field is required",
                    client_req_id=client_req_id,
                    extra={"published": False},
                )
                return
            ok, err, published = await asyncio.to_thread(
                self.node.set_zones_geojson, geojson_payload
            )
            await self._send_ack(
                ws,
                "set_zones_geojson",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"published": bool(published)},
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "load_zones_file":
            ok, err = await asyncio.to_thread(self.node.reload_zones_from_disk)
            await self._send_ack(
                ws,
                "load_zones_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"published": bool(ok)},
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "save_waypoints_file":
            waypoints, _, parse_err = self._parse_waypoints_from_message(msg)
            if waypoints is None:
                await self._send_ack(
                    ws,
                    "save_waypoints_file",
                    False,
                    parse_err,
                    client_req_id=client_req_id,
                )
                return
            ok, err, count = await asyncio.to_thread(self.node.save_waypoints_file, waypoints)
            await self._send_ack(
                ws,
                "save_waypoints_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"waypoint_count": int(count)},
            )
            return

        if op == "load_waypoints_file":
            ok, err, waypoints = await asyncio.to_thread(self.node.load_waypoints_file)
            await self._send_ack(
                ws,
                "load_waypoints_file",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "waypoint_count": int(len(waypoints)),
                    "waypoints": list(waypoints) if ok else [],
                },
            )
            return

        if op == "set_goal_ll":
            waypoints, loop_enabled, parse_err = self._parse_waypoints_from_message(msg)
            if waypoints is None:
                await self._send_ack(
                    ws,
                    "set_goal_ll",
                    False,
                    parse_err,
                    client_req_id=client_req_id,
                )
                return
            ok, err, waypoint_count, loop_used = await asyncio.to_thread(
                self.node.set_nav_goals, waypoints, loop_enabled
            )
            await self._send_ack(
                ws,
                "set_goal_ll",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "waypoint_count": int(waypoint_count),
                    "loop": bool(loop_used),
                },
            )
            return

        if op == "cancel_goal":
            ok, err = await asyncio.to_thread(self.node.cancel_nav_goal)
            await self._send_ack(
                ws, "cancel_goal", ok, err, client_req_id=client_req_id
            )
            return

        if op == "brake":
            ok, err = await asyncio.to_thread(self.node.brake_nav)
            await self._send_ack(
                ws, "brake", ok, err, client_req_id=client_req_id
            )
            return

        if op == "set_manual_mode":
            enabled_raw = msg.get("enabled")
            if not isinstance(enabled_raw, bool):
                await self._send_ack(
                    ws,
                    "set_manual_mode",
                    False,
                    "enabled must be boolean",
                    client_req_id=client_req_id,
                )
                return
            ok, err, enabled_after = await asyncio.to_thread(
                self.node.set_manual_mode,
                enabled_raw,
            )
            await self._send_ack(
                ws,
                "set_manual_mode",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"enabled": bool(enabled_after)},
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "set_manual_cmd":
            try:
                linear_x = float(msg["linear_x"])
                angular_z = float(msg["angular_z"])
                brake_pct = int(float(msg.get("brake_pct", 0)))
            except (KeyError, ValueError, TypeError) as exc:
                await self._send_ack(
                    ws,
                    "set_manual_cmd",
                    False,
                    f"invalid parameters: {exc}",
                    client_req_id=client_req_id,
                )
                return
            ok, err = await asyncio.to_thread(
                self.node.set_manual_cmd,
                linear_x,
                angular_z,
                brake_pct,
            )
            await self._send_ack(
                ws,
                "set_manual_cmd",
                ok,
                err,
                client_req_id=client_req_id,
            )
            return

        if op == "get_nav_snapshot":
            ok, err, payload = await asyncio.to_thread(self.node.get_nav_snapshot)
            if client_req_id is not None:
                payload = dict(payload)
                payload["client_req_id"] = client_req_id
            if ok:
                await self._send_json(ws, payload)
                return
            await self._send_json(
                ws,
                {
                    "op": "nav_snapshot",
                    "ok": False,
                    "error": err or "snapshot request failed",
                    "client_req_id": client_req_id,
                },
            )
            return

        if op == "start_rosbag":
            profile = str(msg.get("profile", "core") or "core")
            ok, err, status_payload = await asyncio.to_thread(self.node.start_rosbag, profile)
            await self._send_ack(
                ws,
                "start_rosbag",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"rosbag": status_payload},
            )
            return

        if op == "stop_rosbag":
            ok, err, status_payload = await asyncio.to_thread(self.node.stop_rosbag)
            await self._send_ack(
                ws,
                "stop_rosbag",
                ok,
                err,
                client_req_id=client_req_id,
                extra={"rosbag": status_payload},
            )
            return

        if op == "camera_pan":
            angle_raw = msg.get("angle")
            try:
                angle = float(angle_raw)
            except (ValueError, TypeError):
                await self._send_ack(
                    ws,
                    "camera_pan",
                    False,
                    "angle must be numeric",
                    client_req_id=client_req_id,
                )
                return
            if not np.isfinite(angle):
                await self._send_ack(
                    ws,
                    "camera_pan",
                    False,
                    "angle must be finite",
                    client_req_id=client_req_id,
                )
                return
            ok, err, _ = await asyncio.to_thread(self.node.camera_pan, angle)
            await self._send_ack(
                ws, "camera_pan", ok, err, client_req_id=client_req_id
            )
            return

        if op == "camera_zoom_toggle":
            ok, err = await asyncio.to_thread(self.node.camera_zoom_toggle)
            await self._send_ack(
                ws,
                "camera_zoom_toggle",
                ok,
                err,
                client_req_id=client_req_id,
            )
            return

        if op == "get_camera_status":
            _, _, payload = await asyncio.to_thread(self.node.get_camera_status)
            if client_req_id is not None:
                payload = dict(payload)
                payload["client_req_id"] = client_req_id
            await self._send_json(ws, payload)
            return

        await self._send_ack(
            ws,
            str(op),
            False,
            "unknown op",
            client_req_id=client_req_id,
            extra={"published": False},
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
        await asyncio.to_thread(node.close)
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
