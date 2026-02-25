#!/usr/bin/env python3

import json
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav2_msgs.msg import CostmapFilterInfo
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from map_tools.zone_io import ZoneDocument
from map_tools.zone_io import ZoneValidationError
from map_tools.zone_io import load_zone_document
from map_tools.zone_io import save_zone_document
from map_tools.zone_io import validate_document
from map_tools.zone_mask import MaskSpec
from map_tools.zone_mask import rasterize_no_go_zones
from map_tools.zone_projection import ZoneProjection
from map_tools.zone_websocket import ZoneWebSocketBridge


class ZoneServer(Node):
    def __init__(self) -> None:
        super().__init__("zone_server")
        self.declare_parameter("zones_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("marker_topic", "/zone_markers")
        self.declare_parameter("state_topic", "/zones_state")
        self.declare_parameter("set_topic", "/zones_cmd")
        self.declare_parameter("load_on_start", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_state", True)
        self.declare_parameter("state_publish_period_sec", 1.0)
        self.declare_parameter("websocket_enabled", True)
        self.declare_parameter("websocket_host", "0.0.0.0")
        self.declare_parameter("websocket_port", 8765)
        self.declare_parameter("projection_enabled", True)
        self.declare_parameter("from_ll_service", "/fromLL")
        self.declare_parameter("to_ll_service", "/toLL")
        self.declare_parameter("projection_request_timeout_sec", 10.0)
        self.declare_parameter("publish_nav2_mask", True)
        self.declare_parameter("nav2_mask_topic", "/keepout_filter_mask")
        self.declare_parameter("nav2_filter_info_topic", "/costmap_filter_info")
        self.declare_parameter("nav2_filter_type", 0)
        self.declare_parameter("nav2_mask_base", 0.0)
        self.declare_parameter("nav2_mask_multiplier", 1.0)
        self.declare_parameter("mask_resolution", 0.1)
        self.declare_parameter("mask_width", 400)
        self.declare_parameter("mask_height", 400)
        self.declare_parameter("mask_origin_x", -20.0)
        self.declare_parameter("mask_origin_y", -20.0)
        self.declare_parameter("mask_default_value", 0)
        self.declare_parameter("mask_no_go_value", 100)
        self.declare_parameter("mask_auto_resize", True)
        self.declare_parameter("mask_auto_resize_margin_m", 2.0)

        self._zones_file = self.get_parameter("zones_file").get_parameter_value().string_value
        default_frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._document = ZoneDocument(frame_id=default_frame_id, zones=[])

        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        set_topic = self.get_parameter("set_topic").get_parameter_value().string_value

        qos_state = QoSProfile(depth=1)
        qos_state.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_state.reliability = ReliabilityPolicy.RELIABLE

        qos_markers = QoSProfile(depth=1)
        qos_markers.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_markers.reliability = ReliabilityPolicy.RELIABLE

        self._marker_pub = self.create_publisher(MarkerArray, marker_topic, qos_markers)
        self._state_pub = self.create_publisher(String, state_topic, qos_state)
        self._set_sub = self.create_subscription(String, set_topic, self._on_set_zones, 10)
        self._ws_bridge: ZoneWebSocketBridge | None = None
        self._projection: ZoneProjection | None = None
        self._pending_projection_jobs: list[dict[str, Any]] = []
        self._mask_spec = self._read_mask_spec()

        nav2_mask_topic = self.get_parameter("nav2_mask_topic").get_parameter_value().string_value
        nav2_filter_info_topic = self.get_parameter("nav2_filter_info_topic").get_parameter_value().string_value
        qos_map = QoSProfile(depth=1)
        qos_map.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = ReliabilityPolicy.RELIABLE
        self._mask_pub = self.create_publisher(OccupancyGrid, nav2_mask_topic, qos_map)
        self._filter_info_pub = self.create_publisher(CostmapFilterInfo, nav2_filter_info_topic, qos_map)

        self.create_service(Trigger, "~/load_zones", self._on_load_zones)
        self.create_service(Trigger, "~/save_zones", self._on_save_zones)
        self.create_service(Trigger, "~/clear_zones", self._on_clear_zones)

        publish_period = self.get_parameter("state_publish_period_sec").get_parameter_value().double_value
        self.create_timer(max(0.2, publish_period), self._publish_state_if_enabled)
        self.create_timer(0.05, self._drain_websocket_commands)
        self.create_timer(0.05, self._poll_projection_jobs)

        projection_enabled = self.get_parameter("projection_enabled").get_parameter_value().bool_value
        if projection_enabled:
            from_ll_service = self.get_parameter("from_ll_service").get_parameter_value().string_value
            to_ll_service = self.get_parameter("to_ll_service").get_parameter_value().string_value
            self._projection = ZoneProjection(self, from_ll_service=from_ll_service, to_ll_service=to_ll_service)
            status = self._projection.status
            if status.ok:
                self.get_logger().info(
                    f"Projection bridge enabled (fromLL='{from_ll_service}', toLL='{to_ll_service}')"
                )
            else:
                self.get_logger().warn(f"Projection bridge disabled: {status.message}")

        websocket_enabled = self.get_parameter("websocket_enabled").get_parameter_value().bool_value
        if websocket_enabled:
            websocket_host = self.get_parameter("websocket_host").get_parameter_value().string_value
            websocket_port = self.get_parameter("websocket_port").get_parameter_value().integer_value
            self._ws_bridge = ZoneWebSocketBridge(host=websocket_host, port=int(websocket_port))
            if not self._ws_bridge.available:
                self.get_logger().error(
                    "websocket_enabled=true but python websockets package is not available"
                )
                self._ws_bridge = None
            else:
                started = self._ws_bridge.start()
                if started:
                    self.get_logger().info(f"WebSocket server listening on ws://{websocket_host}:{websocket_port}")
                else:
                    self.get_logger().error("WebSocket server failed to start")
                    self._ws_bridge = None

        load_on_start = self.get_parameter("load_on_start").get_parameter_value().bool_value
        if load_on_start and self._zones_file:
            loaded = self._try_load_from_disk()
            if not loaded:
                self.get_logger().warn("Zone file load failed, using empty zone document")

        self._publish_all()
        self.get_logger().info("map_tools zone_server started")

    def _on_set_zones(self, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid JSON in set message: {exc}")
            return

        if not isinstance(raw, dict):
            self.get_logger().error("Set message root must be a JSON object")
            return

        updated = self._apply_raw_document(raw, source="ros_topic")
        if updated:
            self.get_logger().info(f"Zone document updated: {len(self._document.zones)} zones")

    def _on_load_zones(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._zones_file:
            response.success = False
            response.message = "zones_file parameter is empty"
            return response
        loaded = self._try_load_from_disk()
        response.success = loaded
        response.message = "Zone file loaded" if loaded else "Failed to load zone file"
        return response

    def _on_save_zones(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._zones_file:
            response.success = False
            response.message = "zones_file parameter is empty"
            return response
        try:
            save_zone_document(self._zones_file, self._document)
        except OSError as exc:
            response.success = False
            response.message = f"Failed to save zone file: {exc}"
            self.get_logger().error(response.message)
            self._push_ws_event(
                {"op": "result", "request": "save", "ok": False, "message": response.message}
            )
            return response
        response.success = True
        response.message = f"Saved {len(self._document.zones)} zones"
        self._push_ws_event({"op": "result", "request": "save", "ok": True, "message": response.message})
        return response

    def _on_clear_zones(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._document = ZoneDocument(frame_id=self._document.frame_id, zones=[])
        self._publish_all()
        self._push_ws_event({"op": "result", "request": "clear", "ok": True, "message": "zones cleared"})
        response.success = True
        response.message = "Cleared zone document"
        return response

    def _try_load_from_disk(self) -> bool:
        try:
            loaded = load_zone_document(self._zones_file)
        except (OSError, ZoneValidationError) as exc:
            self.get_logger().error(f"Failed to load zone file '{self._zones_file}': {exc}")
            return False
        self._document = loaded
        self._publish_all()
        self._push_ws_event(
            {
                "op": "result",
                "request": "load",
                "ok": True,
                "message": f"loaded {len(loaded.zones)} zones",
            }
        )
        self.get_logger().info(f"Loaded {len(loaded.zones)} zones from '{self._zones_file}'")
        return True

    def _publish_state_if_enabled(self) -> None:
        publish_state = self.get_parameter("publish_state").get_parameter_value().bool_value
        if not publish_state:
            return
        self._publish_state()

    def _publish_all(self) -> None:
        publish_markers = self.get_parameter("publish_markers").get_parameter_value().bool_value
        publish_state = self.get_parameter("publish_state").get_parameter_value().bool_value
        if publish_markers:
            self._publish_markers()
        if publish_state:
            self._publish_state()
        publish_nav2_mask = self.get_parameter("publish_nav2_mask").get_parameter_value().bool_value
        if publish_nav2_mask:
            self._publish_nav2_mask()

    def _document_to_dict(self) -> dict[str, object]:
        return {
            "frame_id": self._document.frame_id,
            "zones": [
                {
                    "id": zone.zone_id,
                    "type": zone.zone_type,
                    "enabled": zone.enabled,
                    "polygon": [[point[0], point[1]] for point in zone.polygon],
                }
                for zone in self._document.zones
            ],
        }

    def _apply_raw_document(
        self,
        raw: dict[str, object],
        source: str,
        request_name: str = "set_zones",
    ) -> bool:
        if "frame_id" not in raw:
            raw["frame_id"] = self._document.frame_id
        try:
            new_doc = validate_document(raw)
        except ZoneValidationError as exc:
            self.get_logger().error(f"Zone update rejected ({source}): {exc}")
            self._push_ws_event(
                {
                    "op": "result",
                    "request": request_name,
                    "ok": False,
                    "message": str(exc),
                }
            )
            return False

        self._document = new_doc
        self._publish_all()
        self._push_ws_event(
            {
                "op": "result",
                "request": request_name,
                "ok": True,
                "message": f"updated {len(new_doc.zones)} zones",
            }
        )
        return True

    def _publish_state(self) -> None:
        state = self._document_to_dict()
        self._state_pub.publish(String(data=json.dumps(state, separators=(",", ":"))))
        if self._ws_bridge is not None:
            self._ws_bridge.set_latest_state(state)
            self._ws_bridge.push_event({"op": "zones_state", "document": state})

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = self._document.frame_id
        clear_marker.header.stamp = now
        clear_marker.ns = "zones"
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        marker_id = 1
        for zone in self._document.zones:
            points = [Point(x=point[0], y=point[1], z=0.0) for point in zone.polygon]
            if points:
                points.append(Point(x=points[0].x, y=points[0].y, z=0.0))

            line = Marker()
            line.header.frame_id = self._document.frame_id
            line.header.stamp = now
            line.ns = "zones_outline"
            line.id = marker_id
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.pose.orientation.w = 1.0
            line.points = points
            line.color.r = 1.0
            line.color.g = 0.1 if zone.zone_type == "no_go" else 0.8
            line.color.b = 0.1 if zone.zone_type == "no_go" else 0.1
            line.color.a = 0.95 if zone.enabled else 0.25
            marker_array.markers.append(line)
            marker_id += 1

            centroid_x, centroid_y = _centroid(zone.polygon)
            text = Marker()
            text.header.frame_id = self._document.frame_id
            text.header.stamp = now
            text.ns = "zones_label"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = centroid_x
            text.pose.position.y = centroid_y
            text.pose.position.z = 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.text = f"{zone.zone_id} ({'on' if zone.enabled else 'off'})"
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.90
            marker_array.markers.append(text)
            marker_id += 1

        self._marker_pub.publish(marker_array)

    def _drain_websocket_commands(self) -> None:
        if self._ws_bridge is None:
            return
        while True:
            cmd = self._ws_bridge.pop_command()
            if cmd is None:
                break
            self._handle_ws_command(cmd)

    def _handle_ws_command(self, cmd: dict[str, object]) -> None:
        op = cmd.get("op")
        if not isinstance(op, str):
            self._push_ws_event({"op": "result", "request": "unknown", "ok": False, "message": "missing op"})
            return

        if op == "set_zones":
            raw_doc = cmd.get("document")
            if not isinstance(raw_doc, dict):
                self._push_ws_event(
                    {
                        "op": "result",
                        "request": "set_zones",
                        "ok": False,
                        "message": "document must be an object",
                    }
                )
                return
            self._apply_raw_document(raw_doc, source="websocket")
            return

        if op == "set_zones_ll":
            self._queue_set_zones_ll_command(cmd)
            return

        if op == "get_zones":
            self._push_ws_event({"op": "zones_state", "document": self._document_to_dict()})
            return

        if op == "save":
            if not self._zones_file:
                self._push_ws_event(
                    {"op": "result", "request": "save", "ok": False, "message": "zones_file parameter is empty"}
                )
                return
            try:
                save_zone_document(self._zones_file, self._document)
            except OSError as exc:
                self._push_ws_event(
                    {"op": "result", "request": "save", "ok": False, "message": f"save failed: {exc}"}
                )
                return
            self._push_ws_event({"op": "result", "request": "save", "ok": True, "message": "saved"})
            return

        if op == "load":
            if not self._zones_file:
                self._push_ws_event(
                    {"op": "result", "request": "load", "ok": False, "message": "zones_file parameter is empty"}
                )
                return
            loaded = self._try_load_from_disk()
            if not loaded:
                self._push_ws_event(
                    {"op": "result", "request": "load", "ok": False, "message": "failed to load zone file"}
                )
            return

        if op == "clear":
            self._document = ZoneDocument(frame_id=self._document.frame_id, zones=[])
            self._publish_all()
            self._push_ws_event({"op": "result", "request": "clear", "ok": True, "message": "zones cleared"})
            return

        if op == "ll_to_map":
            self._queue_projection_command(cmd, mode="ll_to_map")
            return

        if op == "map_to_ll":
            self._queue_projection_command(cmd, mode="map_to_ll")
            return

        self._push_ws_event(
            {
                "op": "result",
                "request": op,
                "ok": False,
                "message": "unsupported operation",
            }
        )

    def _push_ws_event(self, event: dict[str, object]) -> None:
        if self._ws_bridge is not None:
            self._ws_bridge.push_event(event)

    def _read_mask_spec(self) -> MaskSpec:
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        resolution = self.get_parameter("mask_resolution").get_parameter_value().double_value
        width = self.get_parameter("mask_width").get_parameter_value().integer_value
        height = self.get_parameter("mask_height").get_parameter_value().integer_value
        origin_x = self.get_parameter("mask_origin_x").get_parameter_value().double_value
        origin_y = self.get_parameter("mask_origin_y").get_parameter_value().double_value
        default_value = self.get_parameter("mask_default_value").get_parameter_value().integer_value
        no_go_value = self.get_parameter("mask_no_go_value").get_parameter_value().integer_value

        return MaskSpec(
            frame_id=frame_id,
            resolution=max(1e-4, float(resolution)),
            width=max(1, int(width)),
            height=max(1, int(height)),
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            default_value=max(0, min(100, int(default_value))),
            no_go_value=max(0, min(100, int(no_go_value))),
        )

    def _publish_nav2_mask(self) -> None:
        active_polygons = [zone.polygon for zone in self._document.zones if zone.enabled and zone.zone_type == "no_go"]
        self._mask_spec = self._effective_mask_spec(active_polygons)
        raster_data = rasterize_no_go_zones(active_polygons, self._mask_spec)

        now = self.get_clock().now().to_msg()
        mask_msg = OccupancyGrid()
        mask_msg.header.frame_id = self._mask_spec.frame_id
        mask_msg.header.stamp = now
        mask_msg.info = MapMetaData()
        mask_msg.info.map_load_time = now
        mask_msg.info.resolution = float(self._mask_spec.resolution)
        mask_msg.info.width = int(self._mask_spec.width)
        mask_msg.info.height = int(self._mask_spec.height)
        mask_msg.info.origin = Pose()
        mask_msg.info.origin.position.x = float(self._mask_spec.origin_x)
        mask_msg.info.origin.position.y = float(self._mask_spec.origin_y)
        mask_msg.info.origin.position.z = 0.0
        mask_msg.info.origin.orientation.w = 1.0
        mask_msg.data = [int(value) for value in raster_data]
        self._mask_pub.publish(mask_msg)

        filter_info = CostmapFilterInfo()
        filter_info.header.frame_id = self._mask_spec.frame_id
        filter_info.header.stamp = now
        filter_info.type = int(self.get_parameter("nav2_filter_type").get_parameter_value().integer_value)
        filter_info.filter_mask_topic = self.get_parameter("nav2_mask_topic").get_parameter_value().string_value
        filter_info.base = float(self.get_parameter("nav2_mask_base").get_parameter_value().double_value)
        filter_info.multiplier = float(
            self.get_parameter("nav2_mask_multiplier").get_parameter_value().double_value
        )
        self._filter_info_pub.publish(filter_info)

    def _effective_mask_spec(self, active_polygons: list[list[tuple[float, float]]]) -> MaskSpec:
        base_spec = self._read_mask_spec()
        auto_resize = self.get_parameter("mask_auto_resize").get_parameter_value().bool_value
        if not auto_resize or not active_polygons:
            return base_spec

        margin = self.get_parameter("mask_auto_resize_margin_m").get_parameter_value().double_value
        margin = max(0.0, float(margin))

        min_x = min(point[0] for polygon in active_polygons for point in polygon) - margin
        max_x = max(point[0] for polygon in active_polygons for point in polygon) + margin
        min_y = min(point[1] for polygon in active_polygons for point in polygon) - margin
        max_y = max(point[1] for polygon in active_polygons for point in polygon) + margin

        span_x = max(base_spec.resolution, max_x - min_x)
        span_y = max(base_spec.resolution, max_y - min_y)
        width = max(base_spec.width, int(span_x / base_spec.resolution) + 1)
        height = max(base_spec.height, int(span_y / base_spec.resolution) + 1)

        return MaskSpec(
            frame_id=base_spec.frame_id,
            resolution=base_spec.resolution,
            width=width,
            height=height,
            origin_x=min_x,
            origin_y=min_y,
            default_value=base_spec.default_value,
            no_go_value=base_spec.no_go_value,
        )

    def _queue_projection_command(self, cmd: dict[str, object], mode: str) -> None:
        if self._projection is None:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": mode,
                    "ok": False,
                    "message": "projection is disabled",
                    "id": cmd.get("id"),
                }
            )
            return

        ready, reason = self._projection.services_ready()
        if not ready:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": mode,
                    "ok": False,
                    "message": reason,
                    "id": cmd.get("id"),
                }
            )
            return

        points = _extract_points(cmd)
        if points is None:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": mode,
                    "ok": False,
                    "message": "point(s) payload is missing or invalid",
                    "id": cmd.get("id"),
                }
            )
            return

        futures: list[Any] = []
        try:
            if mode == "ll_to_map":
                for item in points:
                    lat_raw = item.get("lat", item.get("latitude"))
                    lon_raw = item.get("lon", item.get("longitude", item.get("lng")))
                    if lat_raw is None or lon_raw is None:
                        raise KeyError("lat/lon")
                    lat = float(lat_raw)
                    lon = float(lon_raw)
                    alt = float(item.get("alt", 0.0))
                    future = self._projection.ll_to_map_async(latitude=lat, longitude=lon, altitude=alt)
                    if future is None:
                        raise RuntimeError("projection unavailable")
                    futures.append(future)
            else:
                for item in points:
                    x_raw = item.get("x")
                    y_raw = item.get("y")
                    if x_raw is None or y_raw is None:
                        raise KeyError("x/y")
                    x = float(x_raw)
                    y = float(y_raw)
                    z = float(item.get("z", 0.0))
                    future = self._projection.map_to_ll_async(x=x, y=y, z=z)
                    if future is None:
                        raise RuntimeError("projection unavailable")
                    futures.append(future)
        except (KeyError, TypeError, ValueError) as exc:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": mode,
                    "ok": False,
                    "message": f"invalid point values: {exc}",
                    "id": cmd.get("id"),
                }
            )
            return
        except RuntimeError as exc:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": mode,
                    "ok": False,
                    "message": str(exc),
                    "id": cmd.get("id"),
                }
            )
            return

        timeout_base = self.get_parameter("projection_request_timeout_sec").get_parameter_value().double_value
        # fromLL/toLL services may process many requests sequentially; scale timeout with batch size.
        timeout_sec = max(float(timeout_base), 1.5 + (0.35 * len(futures)))
        job: dict[str, Any] = {
            "op": mode,
            "id": cmd.get("id"),
            "futures": futures,
            "started_sec": self.get_clock().now().nanoseconds / 1e9,
            "timeout_sec": max(0.2, timeout_sec),
        }
        self._pending_projection_jobs.append(job)

    def _queue_set_zones_ll_command(self, cmd: dict[str, object]) -> None:
        if self._projection is None:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": "set_zones_ll",
                    "ok": False,
                    "message": "projection is disabled",
                    "id": cmd.get("id"),
                }
            )
            return

        ready, reason = self._projection.services_ready()
        if not ready:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": "set_zones_ll",
                    "ok": False,
                    "message": reason,
                    "id": cmd.get("id"),
                }
            )
            return

        doc = cmd.get("document")
        if not isinstance(doc, dict):
            self._push_ws_event(
                {
                    "op": "result",
                    "request": "set_zones_ll",
                    "ok": False,
                    "message": "document must be an object",
                    "id": cmd.get("id"),
                }
            )
            return

        frame_id = doc.get("frame_id", self._document.frame_id)
        if not isinstance(frame_id, str) or not frame_id:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": "set_zones_ll",
                    "ok": False,
                    "message": "frame_id must be a non-empty string",
                    "id": cmd.get("id"),
                }
            )
            return

        zones = doc.get("zones", [])
        if not isinstance(zones, list):
            self._push_ws_event(
                {
                    "op": "result",
                    "request": "set_zones_ll",
                    "ok": False,
                    "message": "zones must be a list",
                    "id": cmd.get("id"),
                }
            )
            return

        draft: dict[str, object] = {"frame_id": frame_id, "zones": []}
        index: list[tuple[int, int]] = []
        futures: list[Any] = []

        try:
            for zone_idx, zone in enumerate(zones):
                if not isinstance(zone, dict):
                    raise ValueError(f"zone at index {zone_idx} must be an object")
                zone_id = zone.get("id")
                if not isinstance(zone_id, str) or not zone_id:
                    raise ValueError(f"zone at index {zone_idx} has invalid id")
                zone_type = zone.get("type", "no_go")
                if not isinstance(zone_type, str) or not zone_type:
                    raise ValueError(f"zone '{zone_id}' has invalid type")
                enabled = zone.get("enabled", True)
                if not isinstance(enabled, bool):
                    raise ValueError(f"zone '{zone_id}' has invalid enabled flag")
                ll_points = zone.get("points")
                if not isinstance(ll_points, list) or len(ll_points) < 3:
                    raise ValueError(f"zone '{zone_id}' points must have at least 3 entries")

                cast_zone: dict[str, object] = {
                    "id": zone_id,
                    "type": zone_type,
                    "enabled": enabled,
                    "polygon": [],
                }
                cast_zones = draft["zones"]
                assert isinstance(cast_zones, list)
                cast_zones.append(cast_zone)

                for point_idx, point in enumerate(ll_points):
                    if not isinstance(point, dict):
                        raise ValueError(f"zone '{zone_id}' point at index {point_idx} is invalid")
                    lat_raw = point.get("lat", point.get("latitude"))
                    lon_raw = point.get("lon", point.get("longitude", point.get("lng")))
                    if lat_raw is None or lon_raw is None:
                        raise ValueError(f"zone '{zone_id}' point at index {point_idx} missing lat/lon")
                    lat = float(lat_raw)
                    lon = float(lon_raw)
                    alt = float(point.get("alt", 0.0))
                    future = self._projection.ll_to_map_async(latitude=lat, longitude=lon, altitude=alt)
                    if future is None:
                        raise RuntimeError("projection unavailable")
                    futures.append(future)
                    index.append((zone_idx, point_idx))
        except (ValueError, TypeError, RuntimeError) as exc:
            self._push_ws_event(
                {
                    "op": "result",
                    "request": "set_zones_ll",
                    "ok": False,
                    "message": str(exc),
                    "id": cmd.get("id"),
                }
            )
            return

        timeout_base = self.get_parameter("projection_request_timeout_sec").get_parameter_value().double_value
        timeout_sec = max(float(timeout_base), 1.5 + (0.35 * len(futures)))
        job: dict[str, Any] = {
            "op": "set_zones_ll",
            "id": cmd.get("id"),
            "futures": futures,
            "started_sec": self.get_clock().now().nanoseconds / 1e9,
            "timeout_sec": max(0.2, timeout_sec),
            "draft_document": draft,
            "index": index,
        }
        self._pending_projection_jobs.append(job)

    def _poll_projection_jobs(self) -> None:
        if self._projection is None or not self._pending_projection_jobs:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        completed: list[dict[str, Any]] = []
        for job in self._pending_projection_jobs:
            futures = job["futures"]
            if all(future.done() for future in futures):
                completed.append(job)
                continue
            if now_sec - float(job["started_sec"]) > float(job["timeout_sec"]):
                completed.append(job)
                self._push_ws_event(
                    {
                        "op": "result",
                        "request": job["op"],
                        "ok": False,
                        "message": "projection request timed out",
                        "id": job.get("id"),
                    }
                )

        for job in completed:
            if job in self._pending_projection_jobs:
                self._pending_projection_jobs.remove(job)
            futures = job["futures"]
            if not all(future.done() for future in futures):
                continue

            converted: list[dict[str, float]] = []
            failed: str | None = None
            for future in futures:
                exc = future.exception()
                if exc is not None:
                    failed = str(exc)
                    break
                response = future.result()
                try:
                    if job["op"] in ("ll_to_map", "set_zones_ll"):
                        converted.append(self._projection.parse_from_ll_response(response))
                    else:
                        converted.append(self._projection.parse_to_ll_response(response))
                except (ValueError, TypeError, AttributeError) as exc:
                    failed = f"invalid projection response: {exc}"
                    break

            if failed is not None:
                self._push_ws_event(
                    {
                        "op": "result",
                        "request": job["op"],
                        "ok": False,
                        "message": failed,
                        "id": job.get("id"),
                    }
                )
                continue

            if job["op"] == "set_zones_ll":
                draft = job.get("draft_document")
                index = job.get("index")
                if not isinstance(draft, dict) or not isinstance(index, list):
                    self._push_ws_event(
                        {
                            "op": "result",
                            "request": "set_zones_ll",
                            "ok": False,
                            "message": "internal conversion state is invalid",
                            "id": job.get("id"),
                        }
                    )
                    continue

                try:
                    for i, point in enumerate(converted):
                        ref = index[i]
                        zone_idx, point_idx = ref
                        zones_raw = draft.get("zones")
                        assert isinstance(zones_raw, list)
                        zone_raw = zones_raw[zone_idx]
                        assert isinstance(zone_raw, dict)
                        polygon = zone_raw.get("polygon")
                        assert isinstance(polygon, list)
                        while len(polygon) <= point_idx:
                            polygon.append([0.0, 0.0])
                        polygon[point_idx] = [point["x"], point["y"]]
                except (IndexError, KeyError, TypeError, AssertionError) as exc:
                    self._push_ws_event(
                        {
                            "op": "result",
                            "request": "set_zones_ll",
                            "ok": False,
                            "message": f"failed building projected document: {exc}",
                            "id": job.get("id"),
                        }
                    )
                    continue

                self._apply_raw_document(draft, source="websocket_ll", request_name="set_zones_ll")
                continue

            self._push_ws_event(
                {
                    "op": "projection_result",
                    "request": job["op"],
                    "ok": True,
                    "id": job.get("id"),
                    "points": converted,
                }
            )

    def destroy_node(self) -> bool:
        if self._ws_bridge is not None:
            self._ws_bridge.stop()
        return super().destroy_node()


def _centroid(polygon: list[tuple[float, float]]) -> tuple[float, float]:
    if not polygon:
        return (0.0, 0.0)
    sum_x = 0.0
    sum_y = 0.0
    for point in polygon:
        sum_x += point[0]
        sum_y += point[1]
    return (sum_x / len(polygon), sum_y / len(polygon))


def _extract_points(cmd: dict[str, object]) -> list[dict[str, object]] | None:
    point = cmd.get("point")
    points = cmd.get("points")
    if point is not None:
        if not isinstance(point, dict):
            return None
        return [point]
    if points is not None:
        if not isinstance(points, list):
            return None
        extracted: list[dict[str, object]] = []
        for item in points:
            if not isinstance(item, dict):
                return None
            extracted.append(item)
        return extracted
    return None


def main() -> None:
    rclpy.init()
    node = ZoneServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
