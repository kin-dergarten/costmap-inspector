# costmap-inspector

A Nav2 costmap plugin for identifying which costmap layer is responsible for
lethal cells in a queried area. This allows to figure out **what** blocks a planned path for a robot.

![RViz-style overview of costmap inspection and lethal-cell source attribution](docs/rviz-costmap-inspector-overview.svg)

The overview shows how a merged costmap can be inspected by querying a
footprint against its individual layers, making the source of lethal cells
visible.


This package was build for ROS2 Humble.

## Functionality

`costmap_inspector::LayerInspector` inspects the individual layers of a layered
costmap without modifying the master costmap. It can:

- Query a polygon footprint against every enabled and current costmap layer.
- Count lethal cells per layer and report the layer with the highest count.
- Use either the configured Nav2 costmap footprint or a footprint supplied in
  the request.
- Transform the queried footprint into the costmap frame using TF.
- Persist recently detected lethal layer names for a configurable timeout.
- Publish the lethal cells as a `sensor_msgs/msg/PointCloud2`.
- Optionally publish the checked footprint and individual layers as debug topics.

## Plugin configuration

Add the plugin to the layered costmap's plugin list. The plugin does not add
costs to the master costmap, so it can be placed after the layers it inspects.

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer", "inspector_layer"]

      inspector_layer:
        plugin: "costmap_inspector::LayerInspector"
        enabled: true
        lethal_layers_timeout_seconds: 0.5
        debug.publish_checked_footprint: false
        debug.publish_individual_layers: false
        debug.publish_individual_layers_periodically: false
        debug.publish_periodically_period_seconds: 1.0
```

The layer also supports mapping costmap layer names to human-readable obstacle
sources with `source_names.sources`, `source_names.<source>.name`, and
`source_names.<source>.layers` parameters.

## Query interface

The service is exposed at:

```text
/local_costmap/inspector_layer/polygon_query
```

Service type: `costmap_inspector_msgs/srv/CostmapQuery`.

The request contains:

- `use_costmap_footprint`: use the costmap's configured footprint when true.
- `footprint`: the polygon to query when `use_costmap_footprint` is false.
- `footprint_pose`: pose and frame used to place the footprint.
- `persist_result`: retain layers containing lethal cells for the configured
  timeout.

The service only acknowledges that the request was added to the processing
queue. Results are published asynchronously on:

```text
/local_costmap/inspector_layer/query_result
```

Message type: `costmap_inspector_msgs/msg/CostmapQueryData`. Results include
the number of cells in the polygon, per-layer lethal-cell counts, the most
lethal layer, the mean distance and angle of detected lethal cells, and any
persisted layer or source names. The result status is one of:

- `RESULT_NO_LETHAL_DATA`
- `RESULT_LETHAL_DATA`
- `RESULT_PERSISTED_LETHAL_DATA`
- `RESULT_FAILURE`

## Published topics

- `/local_costmap/inspector_layer/lethal_points` (`sensor_msgs/msg/PointCloud2`)
- `<layer-name>/checked_footprint` (`geometry_msgs/msg/PolygonStamped`) when
  `debug.publish_checked_footprint` is enabled
- `<layer-name>/debug/<costmap-layer>` (`nav_msgs/msg/OccupancyGrid`) when
  individual-layer debug publishing is enabled

## Build and test

Build the packages in a ROS 2 workspace with `colcon`:

```bash
colcon build --packages-up-to costmap_inspector
colcon test --packages-select costmap_inspector
```

The service and message definitions are provided by the companion
`costmap_inspector_msgs` package.
