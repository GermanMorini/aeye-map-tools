# map_tools

ROS 2 package for map-based no-go zone tooling and Nav2 integration.

## Backlog (execution order)
1. Core data model and validation (completed)
- Stable YAML schema in map frame.
- Polygon validation (shape, area, self-intersection).
- Deterministic load/save behavior.
2. Zone runtime server (completed)
- `zone_server` with parameterized topics and file path.
- Dynamic zone updates from JSON topic.
- Load/save/clear services.
- `MarkerArray` + serialized state publishing.
3. Projection bridge (completed)
- Add `zone_projection.py` with `fromLL` and `toLL` clients.
- Normalize all persisted zones to map-frame meters.
4. Nav2 runtime output (completed)
- Rasterize zones into keepout mask `OccupancyGrid`.
- Publish costmap-filter-compatible outputs.
5. Web UI (completed)
- Satellite map editor (draw/edit/delete polygons).
- Open existing zone file and save changes.
- Bridge UI actions to ROS services/topics.
6. Integration and tests (in progress)
- Launch wiring with `navegacion_gps`.
- Unit and runtime checks for deterministic behavior.

## Implemented now
- `map_tools/zone_io.py`:
- Schema validation and zone document load/save.
- `map_tools/zone_server.py`:
- Parameters: `zones_file`, `frame_id`, `marker_topic`, `state_topic`, `set_topic`.
- Services: `~/load_zones`, `~/save_zones`, `~/clear_zones` (`std_srvs/Trigger`).
- Topic input: `set_topic` receives full zone document as JSON string.
- Outputs: `MarkerArray` and serialized current document.
- Launch file: `launch/zone_server.launch.py`.
- Example config and zone file under `config/`.

## Run
```bash
../../tools/compile-ros.sh map_tools
source ../../install/setup.bash
ros2 launch map_tools zone_server.launch.py \
  zones_file:=/tmp/zones.yaml \
  load_on_start:=false \
  websocket_enabled:=true \
  websocket_host:=0.0.0.0 \
  websocket_port:=8765
```

## Nav2 keepout output (implemented)
`zone_server` now publishes:
- `nav_msgs/OccupancyGrid` keepout mask on `/keepout_filter_mask` (configurable)
- `nav2_msgs/CostmapFilterInfo` on `/costmap_filter_info` (configurable)

Mask values are generated dynamically from enabled `no_go` polygons.

## Dynamic update example
Publish a full document to `/zones_cmd`:
```bash
ros2 topic pub --once /zones_cmd std_msgs/msg/String \
  "{data: '{\"frame_id\":\"map\",\"zones\":[{\"id\":\"dock\",\"type\":\"no_go\",\"enabled\":true,\"polygon\":[[1.0,1.0],[3.0,1.0],[3.0,2.0],[1.0,2.0]]}]}' }"
```

Save current state to file:
```bash
ros2 service call /zone_server/save_zones std_srvs/srv/Trigger "{}"
```

## WebSocket protocol (remote UI)
Connect to `ws://<robot-ip>:8765` and exchange JSON messages.

Messages from UI to server:
- `{"op":"set_zones","document":{...}}`
- `{"op":"set_zones_ll","document":{"frame_id":"map","zones":[{"id":"z1","type":"no_go","enabled":true,"points":[{"lat":...,"lon":...,"alt":0.0}]}]}}`
- `{"op":"get_zones"}`
- `{"op":"save"}`
- `{"op":"load"}`
- `{"op":"clear"}`
- `{"op":"ll_to_map","id":"req-1","point":{"lat":41.4,"lon":2.17,"alt":0.0}}`
- `{"op":"map_to_ll","id":"req-2","point":{"x":10.0,"y":20.0,"z":0.0}}`
- Batch mode is also supported with `points:[...]` instead of `point`.

Recommended path for web editors:
- Send `set_zones_ll` directly and let `zone_server` convert + publish in one step.

Messages from server to UI:
- `{"op":"hello","server":"map_tools.zone_server","version":1}`
- `{"op":"accepted","request":"set_zones"}`
- `{"op":"zones_state","document":{...}}`
- `{"op":"result","request":"save","ok":true,"message":"saved"}`
- `{"op":"projection_result","request":"ll_to_map","ok":true,"id":"req-1","points":[{"x":...,"y":...,"z":...}]}`

## Minimal web client
A minimal remote UI is included in [web/index.html](/home/gmorini/Documentos/codigo/ros2/workspace/src/map_tools/web/index.html).
Current features:
- Shows map layers (OpenStreetMap and satellite Esri) and existing no-go polygons from server.
- Create/edit/delete polygons with draw tools.
- Toggle enabled/disabled per polygon by clicking a polygon.
- Push edits to server (`set_zones`) and persist (`save`/`load`).

Run it on your laptop (or any PC):
```bash
cd /home/gmorini/Documentos/codigo/ros2/workspace/src/map_tools/web
python3 -m http.server 8080
```

Then open `http://<your-pc-ip>:8080` and connect to `ws://<robot-ip>:8765`.
Note: the browser needs internet access to load Leaflet libraries and map tiles.

## Integration with `navegacion_gps` (point 5)
1. Apply keepout filter params from [config/nav2_keepout_patch.yaml](/home/gmorini/Documentos/codigo/ros2/workspace/src/map_tools/config/nav2_keepout_patch.yaml) into your Nav2 params file (usually `navegacion_gps/config/nav2_no_map_params.yaml`).
2. Launch integrated stack using [launch/navegacion_gps_with_map_tools.launch.py](/home/gmorini/Documentos/codigo/ros2/workspace/src/map_tools/launch/navegacion_gps_with_map_tools.launch.py):
```bash
ros2 launch map_tools navegacion_gps_with_map_tools.launch.py \
  zones_file:=/tmp/zones.yaml \
  params_file:=/home/gmorini/Documentos/codigo/ros2/workspace/src/navegacion_gps/config/nav2_no_map_params.yaml
```
3. Ensure Nav2 filter topics match:
- `keepout_filter.filter_info_topic: /costmap_filter_info`
- `zone_server.nav2_mask_topic: /keepout_filter_mask`
- `zone_server.nav2_filter_info_topic: /costmap_filter_info`
