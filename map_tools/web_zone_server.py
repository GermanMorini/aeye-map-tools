import asyncio
import base64
from collections import deque
import contextlib
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
from geometry_msgs.msg import TwistStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import Trigger
import yaml

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

try:
    from mavros_msgs.msg import GPSRAW
except ImportError:
    GPSRAW = None

from interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry
from interfaces.srv import (
    BrakeNav,
    CameraPan,
    CameraStatus,
    CancelNavGoal,
    GetDatum,
    GetNavSnapshot,
    GetNavState,
    GetZonesState,
    SetManualMode,
    SetNavGoalLL,
    SetDatum,
    SetZonesGeoJson,
)
try:
    from interfaces.srv import SetControlLock, TouchControlHeartbeat
except ImportError:
    SetControlLock = None
    TouchControlHeartbeat = None
from robot_localization.srv import ToLL
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
SENSOR_INFO_TABS = ("general", "topics", "pixhawk_gps", "lidar", "camera")
FIX_TYPE_NAMES: Dict[int, str] = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D_FIX",
    3: "3D_FIX",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
}
FIX_PRECISION_M: Dict[int, float] = {
    3: 3.0,
    4: 0.5,
    5: 0.3,
    6: 0.02,
}
TOPICS_HISTORY_MAX_MESSAGES = 200
TOPICS_HISTORY_MAX_TEXT_BYTES = 512 * 1024
TOPICS_MAX_ARRAY_ITEMS = 100
TOPICS_MAX_BYTES_PREVIEW = 256


def _stamp_to_dict(stamp: Any) -> Dict[str, int]:
    return {
        "sec": int(getattr(stamp, "sec", 0)),
        "nanosec": int(getattr(stamp, "nanosec", 0)),
    }


def _stamp_to_epoch_ms(stamp: Any) -> Optional[int]:
    sec = getattr(stamp, "sec", None)
    if sec is None:
        return None
    try:
        sec_i = int(sec)
        nanosec_i = int(getattr(stamp, "nanosec", 0))
    except (TypeError, ValueError):
        return None
    return (sec_i * 1000) + int(nanosec_i / 1_000_000)


def _normalize_angle_rad(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _yaw_enu_from_quaternion(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float | None, float | None]:
    values = (qx, qy, qz, qw)
    if not all(math.isfinite(v) for v in values):
        return None, None

    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    yaw = _normalize_angle_rad(yaw)
    yaw_deg = math.degrees(yaw)
    return yaw, yaw_deg


def _fix_quality_class(fix_type_name: str) -> str:
    normalized = str(fix_type_name or "UNKNOWN").strip().upper()
    if normalized == "RTK_FIXED":
        return "good"
    if normalized in {"RTK_FLOAT", "DGPS"}:
        return "warn"
    return "bad"


def _estimated_precision_for_fix(fix_type: Any) -> Optional[float]:
    try:
        numeric = int(fix_type)
    except (TypeError, ValueError):
        return None
    return FIX_PRECISION_M.get(numeric)


def _resolve_fix_type_name(fix_type: Any, explicit_name: Any = None) -> str:
    if explicit_name not in (None, ""):
        return str(explicit_name)
    try:
        numeric = int(fix_type)
    except (TypeError, ValueError):
        return "UNKNOWN"
    return FIX_TYPE_NAMES.get(numeric, "UNKNOWN")


def _finite_or_none(value: Any) -> Optional[float]:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def _json_clone_or_empty(value: Any) -> Dict[str, Any]:
    if isinstance(value, dict):
        return json.loads(json.dumps(value))
    return {}


def _truncate_topic_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _truncate_topic_value(item) for key, item in value.items()}

    if isinstance(value, (list, tuple)):
        items = [_truncate_topic_value(item) for item in list(value)[:TOPICS_MAX_ARRAY_ITEMS]]
        if len(value) > TOPICS_MAX_ARRAY_ITEMS:
            items.append(
                f"... truncated {len(value) - TOPICS_MAX_ARRAY_ITEMS} additional items ..."
            )
        return items

    if isinstance(value, (bytes, bytearray)):
        preview = bytes(value[:TOPICS_MAX_BYTES_PREVIEW]).hex()
        if len(value) > TOPICS_MAX_BYTES_PREVIEW:
            return (
                f"<bytes len={len(value)} preview_hex={preview} "
                f"truncated={len(value) - TOPICS_MAX_BYTES_PREVIEW}>"
            )
        return f"<bytes len={len(value)} hex={preview}>"

    return value


def _topic_message_to_text(msg: Any) -> str:
    payload = message_to_ordereddict(msg)
    truncated = _truncate_topic_value(payload)
    return yaml.safe_dump(
        truncated,
        allow_unicode=False,
        sort_keys=False,
        default_flow_style=False,
    ).strip()


class SensorInfoSession:
    @staticmethod
    def normalize_tab(tab: Any) -> Optional[str]:
        if tab is None:
            return None
        normalized = str(tab).strip().lower()
        if normalized in SENSOR_INFO_TABS:
            return normalized
        return None

    @staticmethod
    def is_implemented_tab(tab: Optional[str]) -> bool:
        return tab in {"general", "topics", "pixhawk_gps"}

    def __init__(self, node: "WebZoneServerNode", ws: Any):
        self.node = node
        self.ws = ws
        self._lock = threading.Lock()
        self._enabled = False
        self._active_tab: Optional[str] = None
        self._interval_s = 0.1
        self._subscriptions: List[Any] = []
        self._data: Dict[str, Any] = {}
        self._topic_name: Optional[str] = None
        self._topic_type: Optional[str] = None
        self._topic_message_type: Any = None
        self._topic_history: deque[Dict[str, Any]] = deque()
        self._topic_history_text_bytes = 0
        self._topic_history_text = ""
        self._topic_truncated = False
        self._topic_error = ""
        self._sender_task: Optional[asyncio.Task[Any]] = None

    def configure(
        self,
        *,
        enabled: bool,
        tab: Optional[str],
        interval_s: float,
        topic_name: Optional[str] = None,
    ) -> Dict[str, Any]:
        normalized_tab = self.normalize_tab(tab)
        clamped_interval = min(5.0, max(0.1, float(interval_s)))
        implemented = self.is_implemented_tab(normalized_tab)
        normalized_topic_name = str(topic_name or "").strip() or None

        with self._lock:
            self._enabled = bool(enabled)
            self._active_tab = normalized_tab if enabled else None
            self._interval_s = clamped_interval
            self._destroy_subscriptions_locked()
            self._data = {}
            self._reset_topics_locked()
            self._cancel_sender_locked()

            if not enabled or normalized_tab is None:
                return {
                    "enabled": False,
                    "tab": None,
                    "implemented": False,
                    "interval_s": clamped_interval,
                }

            if implemented:
                self._create_subscriptions_locked(normalized_tab, normalized_topic_name)

            loop = asyncio.get_running_loop()
            self._sender_task = loop.create_task(
                self._sender_loop(normalized_tab, clamped_interval, implemented)
            )

        payload = {
            "enabled": True,
            "tab": normalized_tab,
            "implemented": implemented,
            "interval_s": clamped_interval,
        }
        if normalized_tab == "topics":
            payload["topic_name"] = normalized_topic_name
            payload["selected_type"] = self._topic_type
        return payload

    def close(self) -> None:
        with self._lock:
            self._enabled = False
            self._active_tab = None
            self._data = {}
            self._reset_topics_locked()
            self._destroy_subscriptions_locked()
            self._cancel_sender_locked()

    def subscription_count(self) -> int:
        with self._lock:
            return len(self._subscriptions)

    def _cancel_sender_locked(self) -> None:
        if self._sender_task is None:
            return
        self.node._loop.call_soon_threadsafe(self._sender_task.cancel)
        self._sender_task = None

    def _destroy_subscriptions_locked(self) -> None:
        for subscription in self._subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception:
                continue
        self._subscriptions = []
        self._topic_message_type = None

    def _create_subscription_locked(
        self, msg_type: Any, topic: str, callback: Any, qos: Any, *, raw: bool = False
    ) -> None:
        if msg_type is None:
            return
        subscription = self.node.create_subscription(
            msg_type, topic, callback, qos, raw=raw
        )
        self._subscriptions.append(subscription)

    def _create_subscriptions_locked(self, tab: str, topic_name: Optional[str]) -> None:
        if tab == "topics":
            if topic_name:
                self._configure_topics_subscription_locked(topic_name)
            return

        self._create_subscription_locked(
            NavSatFix, self.node.info_gps_topic, self._on_gps, qos_profile_sensor_data
        )
        self._create_subscription_locked(
            Int32, self.node.info_fix_type_topic, self._on_fix_type, 10
        )
        self._create_subscription_locked(
            String, self.node.info_rtk_status_topic, self._on_rtk_status, 10
        )
        self._create_subscription_locked(
            Float32, self.node.info_rtcm_age_topic, self._on_rtcm_age, 10
        )
        self._create_subscription_locked(
            Int32, self.node.info_rtcm_count_topic, self._on_rtcm_count, 10
        )
        self._create_subscription_locked(
            String, self.node.info_rtk_source_status_topic, self._on_rtk_source_status, 10
        )
        if GPSRAW is not None:
            self._create_subscription_locked(
                GPSRAW, self.node.info_gps_raw_topic, self._on_gps_raw, qos_profile_sensor_data
            )

        if tab != "pixhawk_gps":
            return

        self._create_subscription_locked(
            Imu, self.node.info_imu_topic, self._on_imu, qos_profile_sensor_data
        )
        self._create_subscription_locked(
            TwistStamped, self.node.info_velocity_topic, self._on_velocity, qos_profile_sensor_data
        )
        self._create_subscription_locked(
            Odometry, self.node.info_odom_topic, self._on_odom, qos_profile_sensor_data
        )

    def _reset_topics_locked(self) -> None:
        self._topic_name = None
        self._topic_type = None
        self._topic_message_type = None
        self._topic_history = deque()
        self._topic_history_text_bytes = 0
        self._topic_history_text = ""
        self._topic_truncated = False
        self._topic_error = ""

    def _configure_topics_subscription_locked(self, topic_name: str) -> None:
        self._topic_name = str(topic_name)
        self._topic_type = None
        self._topic_message_type = None
        self._topic_history = deque()
        self._topic_history_text_bytes = 0
        self._topic_history_text = ""
        self._topic_truncated = False
        self._topic_error = ""

        catalog = self.node.get_topics_catalog()
        match = next((item for item in catalog if item.get("name") == self._topic_name), None)
        if match is None:
            self._topic_error = "topic not found in graph"
            return

        types = match.get("types") or []
        if len(types) == 0:
            self._topic_error = "topic has no announced types"
            return
        if len(types) > 1:
            self._topic_error = "topic has multiple types; selection is ambiguous"
            return

        topic_type = str(types[0])
        try:
            message_type = get_message(topic_type)
        except Exception as exc:
            self._topic_error = f"failed to resolve topic type: {exc}"
            return

        self._topic_type = topic_type
        self._topic_message_type = message_type
        self._create_subscription_locked(
            message_type,
            self._topic_name,
            self._on_topic_raw,
            qos_profile_sensor_data,
            raw=True,
        )

    def _on_topic_raw(self, raw_msg: bytes) -> None:
        with self._lock:
            message_type = self._topic_message_type
            topic_name = self._topic_name
        if message_type is None or topic_name is None:
            return

        try:
            message = deserialize_message(raw_msg, message_type)
            text = _topic_message_to_text(message)
        except Exception as exc:
            with self._lock:
                self._topic_error = f"failed to decode topic message: {exc}"
            return

        received_at_epoch_ms = int(time.time() * 1000.0)
        block = (
            f"# topic: {topic_name}\n"
            f"# received_at: {datetime.fromtimestamp(received_at_epoch_ms / 1000.0).isoformat()}\n"
            f"{text}"
        )
        size_bytes = len(block.encode("utf-8", errors="replace"))
        with self._lock:
            self._topic_history.appendleft(
                {"received_at_epoch_ms": received_at_epoch_ms, "text": block}
            )
            self._topic_history_text_bytes += size_bytes
            while (
                len(self._topic_history) > TOPICS_HISTORY_MAX_MESSAGES
                or self._topic_history_text_bytes > TOPICS_HISTORY_MAX_TEXT_BYTES
            ):
                removed = self._topic_history.pop()
                self._topic_history_text_bytes -= len(
                    str(removed.get("text", "")).encode("utf-8", errors="replace")
                )
                self._topic_truncated = True
            self._topic_history_text = "\n\n---\n\n".join(
                str(item.get("text", "")) for item in self._topic_history
            )

    async def _sender_loop(self, tab: str, interval_s: float, implemented: bool) -> None:
        try:
            if not implemented:
                await self.node.send_ws_json(
                    self.ws,
                    {
                        "op": "sensor_info",
                        "tab": tab,
                        "ok": True,
                        "implemented": False,
                        "snapshot": {},
                        "interval_s": interval_s,
                    },
                )
                return

            while True:
                payload = await self._build_payload(tab, interval_s)
                await self.node.send_ws_json(self.ws, payload)
                await asyncio.sleep(interval_s)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            with contextlib.suppress(Exception):
                await self.node.send_ws_json(
                    self.ws,
                    {
                        "op": "sensor_info",
                        "tab": tab,
                        "ok": False,
                        "implemented": implemented,
                        "error": str(exc),
                        "snapshot": {},
                        "interval_s": interval_s,
                    },
                )

    async def _build_payload(self, tab: str, interval_s: float) -> Dict[str, Any]:
        if tab == "general":
            snapshot = await asyncio.to_thread(self._build_general_snapshot)
        elif tab == "topics":
            snapshot = await asyncio.to_thread(self._build_topics_snapshot)
        elif tab == "pixhawk_gps":
            snapshot = await asyncio.to_thread(self._build_pixhawk_gps_snapshot)
        else:
            snapshot = {}
        return {
            "op": "sensor_info",
            "tab": tab,
            "ok": True,
            "implemented": True,
            "snapshot": snapshot,
            "interval_s": interval_s,
        }

    def _build_topics_snapshot(self) -> Dict[str, Any]:
        catalog = self.node.get_topics_catalog()
        with self._lock:
            selected_topic = self._topic_name
            selected_type = self._topic_type
            history_entries = list(self._topic_history)
            history_text = str(self._topic_history_text)
            truncated = bool(self._topic_truncated)
            error = str(self._topic_error)

        if selected_topic:
            match = next((item for item in catalog if item.get("name") == selected_topic), None)
            if match is None:
                with self._lock:
                    self._topic_error = "topic disappeared from graph"
                    error = self._topic_error
                    self._destroy_subscriptions_locked()
            else:
                types = match.get("types") or []
                if len(types) == 1:
                    next_type = str(types[0])
                    if selected_type is not None and next_type != selected_type:
                        with self._lock:
                            self._topic_error = "topic type changed while subscribed"
                            error = self._topic_error
                            self._destroy_subscriptions_locked()
                    selected_type = next_type
                else:
                    with self._lock:
                        self._topic_error = (
                            "topic has multiple types; selection is ambiguous"
                            if len(types) > 1
                            else "topic has no announced types"
                        )
                        error = self._topic_error
                        self._destroy_subscriptions_locked()

        return {
            "topics_catalog": catalog,
            "selected_topic": selected_topic,
            "selected_type": selected_type,
            "history_entries": history_entries,
            "history_text": history_text,
            "truncated": truncated,
            "error": error,
        }

    def _build_general_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            gps = _json_clone_or_empty(self._data.get("gps"))
            gps_meta = _json_clone_or_empty(self._data.get("gps_meta"))
            rtk_source_state = _json_clone_or_empty(self._data.get("rtk_source_state"))

        datum = self.node.get_datum_info()
        fix_type = gps_meta.get("fix_type")
        fix_type_name = _resolve_fix_type_name(fix_type, gps_meta.get("fix_type_name"))
        precision_m = _estimated_precision_for_fix(fix_type)
        return {
            "datum": datum,
            "gps": gps,
            "gps_meta": {
                **gps_meta,
                "fix_type_name": fix_type_name,
                "fix_quality_class": _fix_quality_class(fix_type_name),
                "estimated_precision_m": precision_m,
            },
            "rtk_source_state": rtk_source_state,
        }

    def _build_pixhawk_gps_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            imu = _json_clone_or_empty(self._data.get("imu"))
            gps = _json_clone_or_empty(self._data.get("gps"))
            gps_meta = _json_clone_or_empty(self._data.get("gps_meta"))
            velocity = _json_clone_or_empty(self._data.get("velocity"))
            odom = _json_clone_or_empty(self._data.get("odom"))
            rtk_source_state = _json_clone_or_empty(self._data.get("rtk_source_state"))

        diagnostics: Dict[str, Any] = {"yaw_delta_deg": None}
        imu_yaw_rad = _finite_or_none(imu.get("yaw_enu_rad"))
        odom_yaw_rad = _finite_or_none(odom.get("yaw_enu_rad"))
        if imu_yaw_rad is not None and odom_yaw_rad is not None:
            diagnostics["yaw_delta_deg"] = math.degrees(
                _normalize_angle_rad(imu_yaw_rad - odom_yaw_rad)
            )

        gps_meta["fix_type_name"] = _resolve_fix_type_name(
            gps_meta.get("fix_type"),
            gps_meta.get("fix_type_name"),
        )
        gps_meta["fix_quality_class"] = _fix_quality_class(gps_meta["fix_type_name"])
        gps_meta["estimated_precision_m"] = _estimated_precision_for_fix(
            gps_meta.get("fix_type")
        )

        return {
            "imu": imu,
            "gps": gps,
            "gps_meta": gps_meta,
            "velocity": velocity,
            "odom": odom,
            "diagnostics": diagnostics,
            "topics": {
                "imu": self.node.info_imu_topic,
                "gps": self.node.info_gps_topic,
                "velocity": self.node.info_velocity_topic,
                "odom": self.node.info_odom_topic,
                "fix_type": self.node.info_fix_type_topic,
                "rtk_status": self.node.info_rtk_status_topic,
                "rtcm_age": self.node.info_rtcm_age_topic,
                "rtcm_count": self.node.info_rtcm_count_topic,
                "gps_raw": self.node.info_gps_raw_topic,
                "rtk_source_status": self.node.info_rtk_source_status_topic,
            },
            "rtk_source_state": rtk_source_state,
        }

    def _on_gps(self, msg: NavSatFix) -> None:
        with self._lock:
            self._data["gps"] = {
                "stamp": _stamp_to_dict(msg.header.stamp),
                "frame_id": str(msg.header.frame_id),
                "status": int(msg.status.status),
                "service": int(msg.status.service),
                "latitude": float(msg.latitude),
                "longitude": float(msg.longitude),
                "altitude": float(msg.altitude),
                "position_covariance": list(msg.position_covariance),
                "position_covariance_type": int(msg.position_covariance_type),
            }

    def _on_fix_type(self, msg: Int32) -> None:
        fix_type = int(msg.data)
        with self._lock:
            gps_meta = self._data.setdefault("gps_meta", {})
            gps_meta["fix_type"] = fix_type
            gps_meta["fix_type_name"] = FIX_TYPE_NAMES.get(fix_type, "UNKNOWN")

    def _on_rtk_status(self, msg: String) -> None:
        with self._lock:
            gps_meta = self._data.setdefault("gps_meta", {})
            gps_meta["rtk_status"] = str(msg.data)

    def _on_rtcm_age(self, msg: Float32) -> None:
        with self._lock:
            gps_meta = self._data.setdefault("gps_meta", {})
            gps_meta["rtcm_age_s"] = float(msg.data)

    def _on_rtcm_count(self, msg: Int32) -> None:
        with self._lock:
            gps_meta = self._data.setdefault("gps_meta", {})
            gps_meta["rtcm_received_count"] = int(msg.data)

    def _on_gps_raw(self, msg: Any) -> None:
        with self._lock:
            gps_meta = self._data.setdefault("gps_meta", {})
            gps_meta["fix_type"] = int(msg.fix_type)
            gps_meta["fix_type_name"] = FIX_TYPE_NAMES.get(int(msg.fix_type), "UNKNOWN")
            gps_meta["satellites_visible"] = int(msg.satellites_visible)
            gps_meta["eph"] = int(msg.eph)
            gps_meta["epv"] = int(msg.epv)

    def _on_rtk_source_status(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data))
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self._data["rtk_source_state"] = payload

    def _on_imu(self, msg: Imu) -> None:
        yaw_enu_rad, yaw_enu_deg = _yaw_enu_from_quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        with self._lock:
            self._data["imu"] = {
                "stamp": _stamp_to_dict(msg.header.stamp),
                "frame_id": str(msg.header.frame_id),
                "orientation": {
                    "x": float(msg.orientation.x),
                    "y": float(msg.orientation.y),
                    "z": float(msg.orientation.z),
                    "w": float(msg.orientation.w),
                },
                "yaw_enu_rad": yaw_enu_rad,
                "yaw_enu_deg": yaw_enu_deg,
                "angular_velocity": {
                    "x": float(msg.angular_velocity.x),
                    "y": float(msg.angular_velocity.y),
                    "z": float(msg.angular_velocity.z),
                },
                "linear_acceleration": {
                    "x": float(msg.linear_acceleration.x),
                    "y": float(msg.linear_acceleration.y),
                    "z": float(msg.linear_acceleration.z),
                },
            }

    def _on_velocity(self, msg: TwistStamped) -> None:
        with self._lock:
            self._data["velocity"] = {
                "stamp": _stamp_to_dict(msg.header.stamp),
                "frame_id": str(msg.header.frame_id),
                "linear": {
                    "x": float(msg.twist.linear.x),
                    "y": float(msg.twist.linear.y),
                    "z": float(msg.twist.linear.z),
                },
                "angular": {
                    "x": float(msg.twist.angular.x),
                    "y": float(msg.twist.angular.y),
                    "z": float(msg.twist.angular.z),
                },
            }

    def _on_odom(self, msg: Odometry) -> None:
        yaw_enu_rad, yaw_enu_deg = _yaw_enu_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        with self._lock:
            self._data["odom"] = {
                "stamp": _stamp_to_dict(msg.header.stamp),
                "frame_id": str(msg.header.frame_id),
                "child_frame_id": str(msg.child_frame_id),
                "position": {
                    "x": float(msg.pose.pose.position.x),
                    "y": float(msg.pose.pose.position.y),
                    "z": float(msg.pose.pose.position.z),
                },
                "orientation": {
                    "x": float(msg.pose.pose.orientation.x),
                    "y": float(msg.pose.pose.orientation.y),
                    "z": float(msg.pose.pose.orientation.z),
                    "w": float(msg.pose.pose.orientation.w),
                },
                "yaw_enu_rad": yaw_enu_rad,
                "yaw_enu_deg": yaw_enu_deg,
                "linear": {
                    "x": float(msg.twist.twist.linear.x),
                    "y": float(msg.twist.twist.linear.y),
                    "z": float(msg.twist.twist.linear.z),
                },
                "angular": {
                    "x": float(msg.twist.twist.angular.x),
                    "y": float(msg.twist.twist.angular.y),
                    "z": float(msg.twist.twist.angular.z),
                },
            }


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
        self.declare_parameter("robot_heading_topic", "/odometry/global")
        self.declare_parameter("project_odom_to_geodetic", False)
        self.declare_parameter("reload_zones_on_connect", True)
        self.declare_parameter("gps_broadcast_hz", 1.0)
        self.declare_parameter("request_timeout_s", 5.0)
        self.declare_parameter("snapshot_request_timeout_s", 5.0)
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
        self.declare_parameter("nav_set_control_lock_service", "/nav_command_server/set_control_lock")
        self.declare_parameter(
            "nav_touch_control_heartbeat_service",
            "/nav_command_server/touch_control_heartbeat",
        )
        self.declare_parameter("nav_set_datum_service", "/datum_setter/set_datum")
        self.declare_parameter("nav_get_datum_service", "/datum_setter/get_datum")
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
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("velocity_topic", "/velocity")
        self.declare_parameter("fix_type_topic", "/gps/fix_type")
        self.declare_parameter("rtk_status_topic", "/gps/rtk_status")
        self.declare_parameter("rtcm_age_topic", "/gps/rtcm_age_s")
        self.declare_parameter("rtcm_count_topic", "/gps/rtcm_received_count")
        self.declare_parameter("gps_raw_topic", "/mavros_node/gps1/raw")
        self.declare_parameter("rtk_source_status_topic", "/gps/rtk_source/status_json")

        self.ws_host = str(self.get_parameter("ws_host").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.robot_heading_topic = str(self.get_parameter("robot_heading_topic").value)
        self.project_odom_to_geodetic = bool(
            self.get_parameter("project_odom_to_geodetic").value
        )
        self.reload_zones_on_connect = bool(
            self.get_parameter("reload_zones_on_connect").value
        )
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
        self.nav_set_control_lock_service = str(
            self.get_parameter("nav_set_control_lock_service").value
        )
        self.nav_touch_control_heartbeat_service = str(
            self.get_parameter("nav_touch_control_heartbeat_service").value
        )
        self.nav_set_datum_service = str(self.get_parameter("nav_set_datum_service").value)
        self.nav_get_datum_service = str(self.get_parameter("nav_get_datum_service").value)
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
        self.info_gps_topic = str(self.get_parameter("gps_topic").value)
        self.info_odom_topic = str(self.get_parameter("odom_topic").value)
        self.info_imu_topic = str(self.get_parameter("imu_topic").value)
        self.info_velocity_topic = str(self.get_parameter("velocity_topic").value)
        self.info_fix_type_topic = str(self.get_parameter("fix_type_topic").value)
        self.info_rtk_status_topic = str(self.get_parameter("rtk_status_topic").value)
        self.info_rtcm_age_topic = str(self.get_parameter("rtcm_age_topic").value)
        self.info_rtcm_count_topic = str(self.get_parameter("rtcm_count_topic").value)
        self.info_gps_raw_topic = str(self.get_parameter("gps_raw_topic").value)
        self.info_rtk_source_status_topic = str(
            self.get_parameter("rtk_source_status_topic").value
        )
        self.nav_get_datum_service = str(self.get_parameter("nav_get_datum_service").value)

        self._lock = threading.Lock()
        self._ws_clients: Set[Any] = set()
        self._ws_send_locks: Dict[Any, asyncio.Lock] = {}
        self._sensor_info_sessions: Dict[Any, SensorInfoSession] = {}
        self._service_client_group = ReentrantCallbackGroup()

        self._last_robot_pose: Optional[Dict[str, float]] = None
        self._last_robot_heading_deg: Optional[float] = None
        self._last_gps_broadcast_monotonic: Optional[float] = None
        self._datum_lat: Optional[float] = None
        self._datum_lon: Optional[float] = None

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
        self._control_locked = True
        self._control_lock_reason = "STARTUP_LOCKED"
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
        self._robot_heading_sub = self.create_subscription(
            Odometry,
            self.robot_heading_topic,
            self._on_robot_heading_odom,
            qos_profile_sensor_data,
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
            SetZonesGeoJson,
            self.zones_set_geojson_service,
            callback_group=self._service_client_group,
        )
        self._zones_get_state_client = self.create_client(
            GetZonesState,
            self.zones_get_state_service,
            callback_group=self._service_client_group,
        )
        self._zones_reload_client = self.create_client(
            Trigger, self.zones_reload_service, callback_group=self._service_client_group
        )
        self._nav_set_goal_client = self.create_client(
            SetNavGoalLL, self.nav_set_goal_service, callback_group=self._service_client_group
        )
        self._nav_cancel_goal_client = self.create_client(
            CancelNavGoal,
            self.nav_cancel_goal_service,
            callback_group=self._service_client_group,
        )
        self._nav_brake_client = self.create_client(
            BrakeNav, self.nav_brake_service, callback_group=self._service_client_group
        )
        self._nav_set_manual_mode_client = self.create_client(
            SetManualMode,
            self.nav_set_manual_mode_service,
            callback_group=self._service_client_group,
        )
        self._nav_set_control_lock_client = (
            self.create_client(
                SetControlLock, self.nav_set_control_lock_service
            )
            if SetControlLock is not None
            else None
        )
        self._nav_touch_control_heartbeat_client = (
            self.create_client(
                TouchControlHeartbeat, self.nav_touch_control_heartbeat_service
            )
            if TouchControlHeartbeat is not None
            else None
        )
        self._nav_set_datum_client = self.create_client(
            SetDatum,
            self.nav_set_datum_service,
            callback_group=self._service_client_group,
        )
        self._nav_get_datum_client = self.create_client(
            GetDatum,
            self.nav_get_datum_service,
            callback_group=self._service_client_group,
        )
        self._nav_to_ll_client = self.create_client(
            ToLL, "/toLL", callback_group=self._service_client_group
        )
        self._teleop_cmd_pub = self.create_publisher(CmdVelFinal, self.teleop_cmd_topic, 10)
        self._nav_get_state_client = self.create_client(
            GetNavState, self.nav_get_state_service, callback_group=self._service_client_group
        )
        self._nav_snapshot_client = self.create_client(
            GetNavSnapshot, self.nav_snapshot_service, callback_group=self._service_client_group
        )
        self._camera_pan_client = self.create_client(
            CameraPan, self.camera_pan_service, callback_group=self._service_client_group
        )
        self._camera_zoom_toggle_client = self.create_client(
            Trigger,
            self.camera_zoom_toggle_service,
            callback_group=self._service_client_group,
        )
        self._camera_status_client = self.create_client(
            CameraStatus,
            self.camera_status_service,
            callback_group=self._service_client_group,
        )
        self.get_logger().info(
            "Web gateway ready "
            f"(ws={self.ws_host}:{self.ws_port}, zones_set={self.zones_set_geojson_service}, "
            f"goal_set={self.nav_set_goal_service}, snapshot={self.nav_snapshot_service}, "
            f"set_datum={self.nav_set_datum_service}, "
            f"set_control_lock={self.nav_set_control_lock_service}, "
            f"touch_control_heartbeat={self.nav_touch_control_heartbeat_service}, "
            f"get_datum={self.nav_get_datum_service}, "
            f"set_control_lock={self.nav_set_control_lock_service}, "
            f"touch_control_heartbeat={self.nav_touch_control_heartbeat_service}, "
            f"nav_events={self.nav_events_topic}, diagnostics={self.diagnostics_topic}, "
            f"rosbag_dir={self.rosbag_output_dir}, "
            f"camera_pan={self.camera_pan_service}, camera_zoom_toggle={self.camera_zoom_toggle_service}, "
            f"camera_status={self.camera_status_service}, "
            f"get_datum={self.nav_get_datum_service}, "
            f"teleop_topic={self.teleop_cmd_topic}, gps_topic={self.gps_topic}, "
            f"odom_topic={self.odom_topic}, "
            f"robot_heading_topic={self.robot_heading_topic}, "
            f"project_odom_to_geodetic={self.project_odom_to_geodetic}, "
            f"reload_zones_on_connect={self.reload_zones_on_connect})"
        )
        self.get_logger().info(f"Waypoints file path: {self.waypoints_file}")
        if self.project_odom_to_geodetic:
            self.create_timer(1.0, self._refresh_datum_from_service)

    def add_client(self, ws: Any) -> None:
        with self._lock:
            self._ws_clients.add(ws)
            self._ws_send_locks[ws] = asyncio.Lock()
            self._sensor_info_sessions[ws] = SensorInfoSession(self, ws)
            count = len(self._ws_clients)
        self.get_logger().info(f"WS client connected (clients={count})")

    def _engage_lock_if_no_clients(self) -> None:
        with self._lock:
            no_clients = len(self._ws_clients) == 0
        if not no_clients:
            return
        ok, err, locked_after = self.set_control_lock(True)
        if ok:
            self.get_logger().info(
                f"Control lock engaged because no WS clients remain (locked={locked_after})"
            )
        else:
            self.get_logger().warning(
                f"Failed to engage control lock after last WS disconnect: {err}"
            )

    def remove_client(self, ws: Any) -> None:
        with self._lock:
            sensor_info_session = self._sensor_info_sessions.pop(ws, None)
            self._ws_clients.discard(ws)
            self._ws_send_locks.pop(ws, None)
            count = len(self._ws_clients)
        if sensor_info_session is not None:
            sensor_info_session.close()
        self.get_logger().info(f"WS client disconnected (clients={count})")
        if count == 0:
            self._engage_lock_if_no_clients()

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

    def set_sensor_info_view(
        self,
        ws: Any,
        enabled: bool,
        tab: Optional[str],
        interval_s: float,
        topic_name: Optional[str] = None,
    ) -> Tuple[bool, str, Dict[str, Any]]:
        if not isinstance(enabled, bool):
            return False, "enabled must be boolean", {}
        try:
            interval = float(interval_s)
        except (TypeError, ValueError):
            return False, "interval_s must be numeric", {}
        if not math.isfinite(interval):
            return False, "interval_s must be finite", {}

        normalized_tab = SensorInfoSession.normalize_tab(tab)
        if enabled and normalized_tab is None:
            return False, "tab must be one of general|topics|pixhawk_gps|lidar|camera", {}
        if (not enabled) and tab is not None and normalized_tab is None:
            return False, "tab must be null or a supported tab when disabling", {}

        with self._lock:
            session = self._sensor_info_sessions.get(ws)
        if session is None:
            return False, "sensor info session unavailable", {}
        payload = session.configure(
            enabled=enabled,
            tab=normalized_tab,
            interval_s=interval,
            topic_name=topic_name,
        )
        payload["subscription_count"] = session.subscription_count()
        return True, "", payload

    def get_topics_catalog(self) -> List[Dict[str, Any]]:
        topics = self.get_topic_names_and_types()
        catalog: List[Dict[str, Any]] = []
        for topic_name, topic_types in topics:
            try:
                publisher_count = int(self.count_publishers(topic_name))
            except Exception:
                publisher_count = None
            try:
                subscriber_count = int(self.count_subscribers(topic_name))
            except Exception:
                subscriber_count = None

            item: Dict[str, Any] = {
                "name": str(topic_name),
                "types": [str(topic_type) for topic_type in list(topic_types or [])],
            }
            if publisher_count is not None:
                item["publisher_count"] = publisher_count
            if subscriber_count is not None:
                item["subscriber_count"] = subscriber_count
            catalog.append(item)
        catalog.sort(key=lambda entry: str(entry.get("name", "")))
        return catalog

    def get_datum_info(self) -> Dict[str, Any]:
        req = GetDatum.Request()
        res = self._call_service(self._nav_get_datum_client, req, self.request_timeout_s)
        if res is None:
            return {
                "ok": False,
                "error": "get_datum timeout",
                "already_set": False,
                "has_current_gps": False,
                "gps_is_rtk": False,
                "current_gps_lat": None,
                "current_gps_lon": None,
                "datum_lat": None,
                "datum_lon": None,
                "last_set_stamp": None,
                "last_set_epoch_ms": None,
                "last_set_source": "",
                "last_set_with_rtk": False,
            }

        last_set_stamp = getattr(res, "last_set_stamp", None)
        current_lat = _finite_or_none(getattr(res, "current_gps_lat", None))
        current_lon = _finite_or_none(getattr(res, "current_gps_lon", None))
        datum_lat = _finite_or_none(getattr(res, "datum_lat", None))
        datum_lon = _finite_or_none(getattr(res, "datum_lon", None))
        return {
            "ok": bool(getattr(res, "ok", False)),
            "error": str(getattr(res, "error", "") or ""),
            "already_set": bool(getattr(res, "already_set", False)),
            "has_current_gps": bool(getattr(res, "has_current_gps", False)),
            "gps_is_rtk": bool(getattr(res, "gps_is_rtk", False)),
            "current_gps_lat": current_lat,
            "current_gps_lon": current_lon,
            "datum_lat": datum_lat,
            "datum_lon": datum_lon,
            "last_set_stamp": _stamp_to_dict(last_set_stamp) if last_set_stamp is not None else None,
            "last_set_epoch_ms": _stamp_to_epoch_ms(last_set_stamp),
            "last_set_source": str(getattr(res, "last_set_source", "") or ""),
            "last_set_with_rtk": bool(getattr(res, "last_set_with_rtk", False)),
        }

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
                "control_locked": bool(self._control_locked),
                "control_lock_reason": str(self._control_lock_reason),
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
            control_locked = bool(self._control_locked)
            control_lock_reason = str(self._control_lock_reason)
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
            "control_locked": control_locked,
            "control_lock_reason": control_lock_reason,
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
        with self._lock:
            sessions = list(self._sensor_info_sessions.values())
            self._sensor_info_sessions = {}
        for session in sessions:
            session.close()

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
            sessions_to_close: List[SensorInfoSession] = []
            remaining = 0
            with self._lock:
                for ws in failed:
                    self._ws_clients.discard(ws)
                    self._ws_send_locks.pop(ws, None)
                    session = self._sensor_info_sessions.pop(ws, None)
                    if session is not None:
                        sessions_to_close.append(session)
                remaining = len(self._ws_clients)
            for session in sessions_to_close:
                session.close()
            if remaining == 0:
                self._engage_lock_if_no_clients()

    def _on_gps_fix(self, msg: NavSatFix) -> None:
        if not np.isfinite(msg.latitude) or not np.isfinite(msg.longitude):
            return

        with self._lock:
            if self._should_project_odom_pose_locked():
                return
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

    def _should_project_odom_pose_locked(self) -> bool:
        return bool(self.project_odom_to_geodetic)

    def _project_map_xy_to_ll_via_service(
        self, x_map_m: float, y_map_m: float, z_map_m: float = 0.0
    ) -> Optional[Tuple[float, float]]:
        if (
            (not np.isfinite(x_map_m))
            or (not np.isfinite(y_map_m))
            or (not np.isfinite(z_map_m))
        ):
            return None
        req = ToLL.Request()
        req.map_point.x = float(x_map_m)
        req.map_point.y = float(y_map_m)
        req.map_point.z = float(z_map_m)
        res = self._call_service(
            self._nav_to_ll_client, req, min(self.request_timeout_s, 1.0)
        )
        if res is None:
            return None
        lat = float(getattr(res.ll_point, "latitude", float("nan")))
        lon = float(getattr(res.ll_point, "longitude", float("nan")))
        if (not np.isfinite(lat)) or (not np.isfinite(lon)):
            return None
        return lat, lon

    def _refresh_datum_from_service(self) -> None:
        if not self.project_odom_to_geodetic:
            return
        if not self._nav_get_datum_client.wait_for_service(timeout_sec=0.05):
            return
        req = GetDatum.Request()
        future = self._nav_get_datum_client.call_async(req)
        res = self._wait_for_future(future, min(self.request_timeout_s, 0.5))
        if res is None or not bool(getattr(res, "ok", False)):
            return
        if not bool(getattr(res, "already_set", False)):
            return
        datum_lat = float(getattr(res, "datum_lat", float("nan")))
        datum_lon = float(getattr(res, "datum_lon", float("nan")))
        if (not np.isfinite(datum_lat)) or (not np.isfinite(datum_lon)):
            return
        with self._lock:
            self._datum_lat = datum_lat
            self._datum_lon = datum_lon

    def _on_robot_heading_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        heading_deg = self._yaw_deg_from_quaternion(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        if heading_deg is None:
            return
        projected_pose = None
        should_broadcast = False
        now = time.monotonic()
        with self._lock:
            self._last_robot_heading_deg = float(heading_deg)
            should_project = self._should_project_odom_pose_locked()
            last_sent = self._last_gps_broadcast_monotonic
        min_interval = 1.0 / max(0.1, float(self.gps_broadcast_hz))
        should_sample_projection = (
            should_project
            and str(msg.header.frame_id or "") == self.map_frame
            and (last_sent is None or (now - last_sent) >= min_interval)
        )
        if should_sample_projection:
            projected_ll = self._project_map_xy_to_ll_via_service(
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            )
            if projected_ll is not None:
                projected_pose = self._build_robot_pose(
                    lat=projected_ll[0],
                    lon=projected_ll[1],
                    heading_deg=heading_deg,
                )
        with self._lock:
            if projected_pose is not None:
                self._last_robot_pose = projected_pose
                self._last_gps_broadcast_monotonic = now
                should_broadcast = True
            elif self._last_robot_pose is not None:
                self._last_robot_pose["heading_deg"] = float(heading_deg)
        if should_broadcast and projected_pose is not None:
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"op": "robot_pose", "pose": projected_pose}),
                self._loop,
            )

    def _on_nav_telemetry(self, msg: NavTelemetry) -> None:
        robot_pose_payload = None
        with self._lock:
            self._cmd_vel_safe = {
                "available": bool(msg.cmd_vel_available),
                "linear_x": float(msg.cmd_vel_linear_x),
                "angular_z": float(msg.cmd_vel_angular_z),
            }
            self._goal_active = bool(msg.goal_active)
            self._control_locked = bool(getattr(msg, "control_locked", False))
            self._control_lock_reason = str(getattr(msg, "control_lock_reason", "") or "")
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

            if (
                (not self._should_project_odom_pose_locked())
                and np.isfinite(msg.robot_lat)
                and np.isfinite(msg.robot_lon)
            ):
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
            self._control_locked = bool(getattr(response, "control_locked", False))
            self._control_lock_reason = str(
                getattr(response, "control_lock_reason", "") or ""
            )
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
            if (
                (not self._should_project_odom_pose_locked())
                and np.isfinite(response.robot_lat)
                and np.isfinite(response.robot_lon)
            ):
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

    @staticmethod
    def _normalize_yaw_deg(yaw_deg: float) -> float:
        yaw = float(yaw_deg)
        while yaw <= -180.0:
            yaw += 360.0
        while yaw > 180.0:
            yaw -= 360.0
        return float(yaw)

    def set_nav_goals(
        self, waypoints: List[Dict[str, float]], loop: bool
    ) -> Tuple[bool, str, int, bool]:
        if len(waypoints) == 0:
            return False, "at least one waypoint is required", 0, False
        with self._lock:
            if self._control_locked:
                return (
                    False,
                    f"control locked: {self._control_lock_reason or 'locked'}",
                    len(waypoints),
                    bool(loop),
                )

        self.get_logger().info(
            f"WS->ROS set_nav_goals (count={len(waypoints)}, loop={bool(loop)})"
        )
        req = SetNavGoalLL.Request()
        loop_enabled = bool(loop and (len(waypoints) > 1))

        if len(waypoints) == 1:
            waypoint = waypoints[0]
            req.lats = []
            req.lons = []
            req.yaws_deg = []
            req.lat = float(waypoint["lat"])
            req.lon = float(waypoint["lon"])
            req.yaw_deg = self._normalize_yaw_deg(float(waypoint.get("yaw_deg", 0.0)))
        else:
            req.lats = [float(wp["lat"]) for wp in waypoints]
            req.lons = [float(wp["lon"]) for wp in waypoints]
            req.yaws_deg = [
                self._normalize_yaw_deg(float(wp.get("yaw_deg", 0.0)))
                for wp in waypoints
            ]
            # Keep legacy single-goal fields populated for compatibility.
            req.lat = float(req.lats[0])
            req.lon = float(req.lons[0])
            req.yaw_deg = float(req.yaws_deg[0])
        req.loop = loop_enabled

        # Multiple LL->map conversions plus Nav2 goal dispatch can take longer than
        # the generic request timeout, especially when datum/fromLL is still warming up.
        goal_timeout_s = max(float(self.set_goal_timeout_s), 6.0 + (2.5 * float(len(waypoints))))
        res = self._call_service(self._nav_set_goal_client, req, goal_timeout_s)
        if res is None:
            return (
                False,
                f"set_goal_ll timeout after {goal_timeout_s:.1f}s",
                len(waypoints),
                loop_enabled,
            )
        if not res.ok:
            self.get_logger().warning(f"set_nav_goals failed: {res.error}")
        else:
            self.get_logger().info("set_nav_goals ok")
        return bool(res.ok), str(res.error), len(waypoints), loop_enabled

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
        if bool(enabled):
            with self._lock:
                if self._control_locked:
                    return False, f"control locked: {self._control_lock_reason or 'locked'}", False
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
        with self._lock:
            if self._control_locked:
                return False, f"control locked: {self._control_lock_reason or 'locked'}"

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

    def set_control_lock(self, locked: bool) -> Tuple[bool, str, bool]:
        if SetControlLock is None or self._nav_set_control_lock_client is None:
            return False, "set_control_lock unsupported by installed interfaces package", bool(locked)
        req = SetControlLock.Request()
        req.locked = bool(locked)
        res = self._call_service(self._nav_set_control_lock_client, req, self.request_timeout_s)
        if res is None:
            return False, "set_control_lock timeout", bool(locked)
        locked_after = bool(getattr(res, "locked_after", locked))
        with self._lock:
            self._control_locked = locked_after
            self._control_lock_reason = "UI_LOCK_REQUEST" if locked_after else ""
        if res.ok:
            self.get_nav_state()
        return bool(res.ok), str(res.error), locked_after

    def touch_control_heartbeat(self) -> Tuple[bool, str, bool]:
        if TouchControlHeartbeat is None or self._nav_touch_control_heartbeat_client is None:
            return False, "control_heartbeat unsupported by installed interfaces package", True
        req = TouchControlHeartbeat.Request()
        res = self._call_service(
            self._nav_touch_control_heartbeat_client,
            req,
            self.request_timeout_s,
        )
        if res is None:
            return False, "control_heartbeat timeout", True
        locked = bool(getattr(res, "locked", False))
        with self._lock:
            self._control_locked = locked
            if locked and not self._control_lock_reason:
                self._control_lock_reason = "LOCKED"
            if not locked:
                self._control_lock_reason = ""
        return bool(res.ok), str(getattr(res, "error", "")), locked

    def set_datum_current(self) -> Tuple[bool, str]:
        req = SetDatum.Request()
        req.coords = []
        res = self._call_service(self._nav_set_datum_client, req, self.request_timeout_s)
        if res is None:
            return False, "set_datum timeout"

        ok = bool(getattr(res, "ok", False))
        error = str(getattr(res, "error", "") or "")
        status_message = str(getattr(res, "status_message", "") or "")
        if ok:
            self._refresh_datum_from_service()
            return True, status_message
        if error:
            return False, error
        if status_message:
            return False, status_message
        return False, "set_datum failed"

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
            if self.node.reload_zones_on_connect:
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
        if op not in {"set_manual_cmd", "control_heartbeat"}:
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

        if op == "set_control_lock":
            locked_raw = msg.get("locked")
            if not isinstance(locked_raw, bool):
                await self._send_ack(
                    ws,
                    "set_control_lock",
                    False,
                    "locked must be boolean",
                    client_req_id=client_req_id,
                )
                return
            ok, err, locked_after = await asyncio.to_thread(
                self.node.set_control_lock,
                locked_raw,
            )
            await self._send_ack(
                ws,
                "set_control_lock",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "locked": bool(locked_after),
                    "control_locked": bool(locked_after),
                    "control_lock_reason": self.node.snapshot_state().get(
                        "control_lock_reason", ""
                    ),
                },
            )
            if ok:
                await self.node._broadcast(self.node.snapshot_state())
            return

        if op == "control_heartbeat":
            ok, err, locked = await asyncio.to_thread(self.node.touch_control_heartbeat)
            await self._send_ack(
                ws,
                "control_heartbeat",
                ok,
                err,
                client_req_id=client_req_id,
                extra={
                    "locked": bool(locked),
                    "control_locked": bool(locked),
                    "control_lock_reason": self.node.snapshot_state().get(
                        "control_lock_reason", ""
                    ),
                },
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

        if op == "set_datum":
            ok, err = await asyncio.to_thread(self.node.set_datum_current)
            await self._send_ack(
                ws,
                "set_datum",
                ok,
                err,
                client_req_id=client_req_id,
            )
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

        if op == "set_sensor_info_view":
            enabled_raw = msg.get("enabled")
            interval_raw = msg.get("interval_s", 0.1)
            tab_raw = msg.get("tab")
            topic_name_raw = msg.get("topic_name")
            ok, err, payload = self.node.set_sensor_info_view(
                ws,
                enabled=enabled_raw,
                tab=tab_raw,
                interval_s=interval_raw,
                topic_name=topic_name_raw,
            )
            await self._send_ack(
                ws,
                "set_sensor_info_view",
                ok,
                err,
                client_req_id=client_req_id,
                extra=payload,
            )
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
