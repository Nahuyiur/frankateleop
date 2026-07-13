# Task Data Validation

Validate one task folder under the NAS recording root without touching robot-control code.

```bash
bash V_validate_task_data.sh put_jenga_drawer
bash V_validate_task_data.sh put_jenga_drawer --quality High_Quality --limit 10
bash V_validate_task_data.sh chuyao_data_collection/pick_banana --root /home/pnp/Desktop/Muka_NAS
```

The validator recursively finds episode directories below:

```text
/home/pnp/Desktop/Muka_NAS/<task_name>/
```

It supports current GUI quality folders:

```text
<task>/High_Quality/<index>/
<task>/Low_Quality/<index>/
<task>/Failure/<index>/
```

and older nested forms such as:

```text
<task>/<index>/
<task>/<subtask>/<index>/
```

Checks performed:

- at least three camera views by default;
- v1/v2 embedded-image fields or v3 video-backed camera metadata consistency;
- pkl frame count, metadata frame count, and per-camera mp4 frame count alignment;
- v3 per-camera MP4 width and height against `metadata.json.image_storage`;
- pkl timestamp monotonicity, cadence, FPS, large gaps, and video-duration consistency;
- single-arm and dual-arm action fields, finite values, joint shape, joint jumps, and joint velocity;
- gripper width bounds;
- optional robot timing fields such as `robot_state_age_ms` and `robot_read_duration_ms`.

Important limitation: current recordings do not store each RealSense frame's hardware timestamp.
This tool verifies saved frame-level alignment between multi-view video and action data; it cannot
prove camera hardware timestamp synchronization after the fact.
