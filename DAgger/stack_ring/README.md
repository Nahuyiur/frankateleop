# Stack Ring mapping and real-robot hover validation

This directory mirrors the proven Jenga split between **mapping** and
**execution**, but uses the calibrated Stack-Ring detector/model and a safer
relative-to-source correction:

```text
stack_ring/
├── stack_ring_retarget.py
├── stack_ring_hover_validate.py
├── run_stack_ring_mapping.sh
├── run_stack_ring_hover.sh
├── calibration/
│   ├── ring_calibration_v1.py
│   ├── ring_mapping_model_selected.json
│   └── ring_mapping_report.json
└── tests/
    └── test_stack_ring_robot_validation.py
```

```text
target_pick_xy  = demo_pick_xy
                + map(current_pick_pixel) - map(source_pick_pixel)
target_place_xy = demo_place_xy
                + map(current_place_pixel) - map(source_place_pixel)
```

Fixed mapping bias therefore cancels. Z, RPY, timestamps, and gripper fields
come from the successful reference trajectory. The pickup/place offset is
interpolated between Ring interaction anchors.

## Safety boundary

- `stack_ring_retarget.py` has no robot client and cannot move hardware.
- `stack_ring_hover_validate.py` is read-only by default.
- First validation only moves above pick/place with at least 8 cm clearance.
- It never closes the gripper and never descends to grasp/release Z.
- Motion requires both `--execute` and `--confirm STACK_RING_HOVER`.
- Full grasp/stack replay is intentionally not enabled in this first gate.
- Reports explicitly record `approved_execution_scope=hover_only` and
  `full_replay_eligible=false`; the hover tool rejects any other scope.

## Offline zero-change regression

Use the same episode as reference and current scene. The mapped pick/place
offsets must be numerically zero.

```bash
bash run_stack_ring_mapping.sh \
  --demo /path/to/stack_ring/High_Quality/0/0.pkl.gz \
  --current-episode /path/to/stack_ring/High_Quality/0/0.pkl.gz \
  --output-dir /tmp/stack_ring_zero_regression
```

The reader supports current v3 video-first episodes through companion
`left.mp4`, old embedded-image PKL files, explicit PNG/JPEG, and live RealSense.

## Live mapping only

Run from the `frankateleop` capture environment:

```bash
bash run_stack_ring_mapping.sh \
  --demo ~/Desktop/Muka_NAS/stack_ring/High_Quality/0/0.pkl.gz \
  --capture-live \
  --output-dir ~/Desktop/Muka_NAS/stack_ring/DAgger/live_validation
```

Inspect before any robot connection:

```text
source_ring_detections.png
current_ring_detections.png
source_trajectory_overlay.png
mapped_trajectory_overlay.png
source_vs_mapped_contact_sheet.png
mapping_report.json
```

## Robot-node read-only preflight

The node on port 6002 must eventually be Cartesian `control_mode=ee`, but the
following command only reads mode/current pose and writes a hover plan:

```bash
bash run_stack_ring_hover.sh \
  --mapping-report ~/Desktop/Muka_NAS/stack_ring/DAgger/live_validation/mapping_report.json \
  --target pick \
  --host 127.0.0.1 --port 6002
```

## Explicit hover execution

Only after reviewing the mapping images and preflight plan, clearing the robot
workspace, confirming the emergency stop, and verifying `control_mode=ee`:

```bash
bash run_stack_ring_hover.sh \
  --mapping-report ~/Desktop/Muka_NAS/stack_ring/DAgger/live_validation/mapping_report.json \
  --target pick \
  --host 127.0.0.1 --port 6002 \
  --execute --confirm STACK_RING_HOVER
```

Validate pick hover first. Place hover is a separate explicit run using
`--target place`. Do not enable full task replay until both hover targets are
visually within the agreed error tolerance.
