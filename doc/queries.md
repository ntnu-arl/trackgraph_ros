# Segment queries

Keep both TrackGraph and the tracker alive. From a fourth sourced terminal:

```bash
ros2 run hydra_ros query_open_vocab_objects \
  --text "armchair" \
  --top-k 5
```

For an image query:

```bash
ros2 run hydra_ros query_open_vocab_objects \
  --image /path/to/potted-plant.jpg \
  --top-k 5
```

Text, image, and saved-feature inputs can also be mixed in one accumulated
visualization, in command-line order:

```bash
ros2 run hydra_ros query_open_vocab_objects \
  --query stairs --top-k 1 \
  --query "red car" --top-k 3 \
  --image /path/to/gasoline-barrel.png --top-k 1
```

`--top-k` sets the maximum number of ranked matches. Each query prints the results,
updates RViz, and publishes their segment centers for robot inspection as a
`geometry_msgs/msg/PoseArray` on
`/hydra/query_open_vocab_objects/inspection_targets`. These poses are not navigation
goals.

Use `--output-json` to save the response, `--min-mesh-vertices N` to ignore
objects with fewer than `N` mesh vertices, or `--stop-mapping` to query a frozen
map. The minimum-vertex threshold retains its existing default of 20 when the
flag is omitted; pass `--min-mesh-vertices 0` to disable the filter. `--query`
is an alias for `--text`.

RViz exposes the object-level query view as three independent displays:

- **OpenVocab Query Objects** contains only selected nodes, glows, and their
  mesh-to-layer connectors.
- **OpenVocab Query Object Context** contains only the remaining washed-out
  object nodes and connectors.
- **OpenVocab Query Object Descriptors** contains only the selected objects'
  query, rank, score, UID, node, and source-track text.

Enable only the first for isolated results, add the second to show queried
objects among the full object layer, and toggle descriptor text independently.


## Visualization topics

| Topic | View |
| --- | --- |
| `/[namespace]/tracking/overlay/image_raw` | RGB with tracked masks |
| `/[namespace]/tracking/masks/image_raw` | Raw track-id mask |
| `/[namespace]/tracking/color/image_raw` | Colored mask; disabled by default |

| Topic | View |
| --- | --- |
| `/hydra/tracked_objects/mesh_surface_overlay` | Tracked mesh; shown by default |
| `/hydra/tracked_objects/mesh_vertices` | Object mesh-support points |
| `/hydra/tracked_objects/mesh_vertex_labels` | Object-id labels |

| Topic | View |
| --- | --- |
| `/hydra/tracked_objects/open_vocab_query_mesh` | Highlighted result mesh; shown by default |
| `/hydra/tracked_objects/open_vocab_query_object_layer` | Highlighted query nodes and connector lines |
| `/hydra/tracked_objects/open_vocab_query_object_context` | Washed-out non-query nodes and connector lines |
| `/hydra/tracked_objects/open_vocab_query_object_descriptors` | Query-result descriptor text at highlighted nodes |
| `/hydra/query_open_vocab_objects/markers` | Result points and labels |
| `/hydra/query_open_vocab_objects/inspection_targets` | Ranked robot-inspection positions |
