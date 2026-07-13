# Jenga DAgger Retargeting

This directory contains a small offline/online retargeting pipeline for the
left-arm Jenga stack task.

## What it does

`jenga_mapping_model.py` builds a validated pixel-to-robot-XY mapping from the
successful `High_Quality` episodes. Each successful trajectory contributes the
detected pick/place block pixels and the matching robot pickup/place XY anchors;
a held-out validation split reports the mapping error before live use.

`jenga_retarget.py` then detects the current pick Jenga and right/place Jenga
from side-view cameras, maps their current pixels through the validated model,
and writes a new `.pkl.gz` episode whose Cartesian `pose[:3]` trajectory is
shifted to those target coordinates. The `--demo` episode is still the output
trajectory template. Joint, timing, and gripper fields are preserved. Embedded
`*_image` fields are stripped by default so the output is small and fast to
replay; pass `--keep-images` only if you need a full debug copy.

The script does not execute the robot. The generated file has retargeted
Cartesian `pose` values while the original `joint` values are kept for metadata.
Do not execute it with `7_replay_fr3.sh`, because script 7 sends joints. Use
`DAgger/replay_jenga_cartesian.sh` for robot execution.

## Default dry run

From `/home/pnp/frankateleop`:

First build and validate the mapping model from a random split of successful demos:

```bash
bash DAgger/build_jenga_mapping_model.sh \
  --model-type affine \
  --train-demos 10 \
  --validation-demos 5 \
  --random-seed 7
```

This writes:

```text
~/Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json
```

Then retarget an offline current frame:

```bash
bash DAgger/run_jenga_retarget.sh \
  --demo ~/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz \
  --mapping-model ~/Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json \
  --current-episode ~/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz \
  --output ~/Desktop/Muka_NAS/stack_jenga/DAgger/retargeted_0.pkl.gz \
  --debug-dir ~/Desktop/Muka_NAS/stack_jenga/DAgger/debug_0
```

You can inspect the generated file without touching the robot:

```bash
bash 7_replay_fr3.sh ~/Desktop/Muka_NAS/stack_jenga/DAgger/retargeted_0.pkl.gz \
  --skip-robot-check
```

For Cartesian dry-run against the robot node:

```bash
bash DAgger/replay_jenga_cartesian.sh ~/Desktop/Muka_NAS/stack_jenga/DAgger/retargeted_0.pkl.gz \
  --host 127.0.0.1 --port 6002 \
  --gripper-host 127.0.0.1 --gripper-port 50054
```

Launch the robot node with `--control-mode ee` before Cartesian execution. Add
`--execute` to the Cartesian replay command only after inspecting the dry-run
output.

If the current `6002` node is in joint mode, stop that node and start the
matching Cartesian node:

```bash
bash DAgger/launch_jenga_ee_node.sh
```

The helper defaults match the current left-arm setup:

```text
robot=fr3_left
tele_port=6002
robot_port=50052
gripper_port=50054
control_mode=ee
```

## Live current scene

Omit `--current-episode` to capture one live RGB frame from the configured
`left,middle` RealSense side cameras:

```bash
bash DAgger/run_jenga_retarget.sh \
  --demo ~/Desktop/Muka_NAS/stack_jenga/High_Quality/0/0.pkl.gz \
  --mapping-model ~/Desktop/Muka_NAS/stack_jenga/DAgger/mapping_model.json \
  --output ~/Desktop/Muka_NAS/stack_jenga/DAgger/live_retargeted.pkl.gz \
  --debug-dir ~/Desktop/Muka_NAS/stack_jenga/DAgger/live_debug
```

Inspect `demo_left.jpg`, `current_left.jpg`, and `mapping_summary.json` in the
debug directory before executing.

The shortest live mapping command is:

```bash
bash DAgger/run_live_jenga_stack.sh
```

It opens the live cameras, warms them up for 3 seconds, requires the validated
mapping model by default, performs live retargeting only, then writes:

```text
~/Desktop/Muka_NAS/stack_jenga/DAgger/live_retargeted.pkl.gz
~/Desktop/Muka_NAS/stack_jenga/DAgger/live_debug/current_left.jpg
~/Desktop/Muka_NAS/stack_jenga/DAgger/live_debug/stack_height_left.jpg
~/Desktop/Muka_NAS/stack_jenga/DAgger/live_debug/mapping_summary.json
```

If you later want a Cartesian replay dry-run, add `--with-replay`.

Live retargeting also adjusts the final place height for repeated stacking.
The place XY remains mapped from the current right-side stack; only
`target_place_xyz[2]` changes. The right-side stack count is estimated from the
left side camera and the per-layer Z increment defaults to:

```text
demo_place_z - demo_pick_z
```

For field correction, override the count or layer height:

```bash
bash DAgger/run_live_jenga_stack.sh --stack-count-override 3
bash DAgger/run_live_jenga_stack.sh --block-height-m 0.031
```

If lighting/exposure still needs more time, increase camera warmup:

```bash
bash DAgger/run_live_jenga_stack.sh --camera-warmup-sec 5
```

If the stack-height detector is too conservative or too aggressive, tune the
quality gate:

```bash
bash DAgger/run_live_jenga_stack.sh --min-stack-height-quality 0.65
```

For the current left-camera view, the stack silhouette is compressed: the
apparent per-layer increment is about 17% of the demo single-block bbox height.
The default uses that calibration. You can tune it directly:

```bash
bash DAgger/run_live_jenga_stack.sh --stack-layer-pitch-ratio 0.17
bash DAgger/run_live_jenga_stack.sh --stack-layer-pitch-px 9.2
```

Disable dynamic height if needed:

```bash
bash DAgger/run_live_jenga_stack.sh --no-dynamic-place-z
```

To optionally rebuild the mapping model with a larger split:

```bash
bash DAgger/build_jenga_mapping_model.sh \
  --model-type affine \
  --train-demos 30 \
  --validation-demos 10 \
  --random-seed 7
```

To intentionally use the legacy two-point mapping fallback, pass
`--no-mapping-model` or `--allow-old-mapping` to `run_live_jenga_stack.sh`.

Then dry-run the live Cartesian replay:

```bash
bash DAgger/replay_jenga_cartesian.sh \
  ~/Desktop/Muka_NAS/stack_jenga/DAgger/live_retargeted.pkl.gz \
  --host 127.0.0.1 --port 6002 \
  --gripper-host 127.0.0.1 --gripper-port 50054
```

Execute only after the node reports `Robot control mode: ee`, the start pose
check is acceptable, and you intentionally opt into replay:

```bash
bash DAgger/run_live_jenga_stack.sh --approach-start --execute
```

## Notes

- The current implementation is intentionally conservative: `left` is the
  primary mapping camera. If `middle` is occluded by the robot, it is logged in
  debug output but skipped for coordinate mapping.
- With the current RGB-only dataset, z is preserved from the demo. If you want
  true 3D triangulation, first calibrate side-camera extrinsics or record RGB-D.
- If the right/place block is not visible, pass `--no-detect-place`; the script
  will retarget pickup and keep the original pickup-to-place offset.
