# franka_downsample

这个模块负责把已有 `franka_capture` 数据按固定 stride 降采样，并重新保存成同样的采集格式。

当前主要用途：

```text
30Hz put_eraser_into_drawer -> 10Hz put_eraser_into_drawer
```

## 使用方式

```bash
bash 12_downsample_task_to_10hz.sh \
  /home/pnp/Desktop/franka_record_data/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer \
  --camera right \
  --source-fps 30 \
  --target-fps 10 \
  --overwrite
```

如果要保留全部相机视角：

```bash
bash 12_downsample_task_to_10hz.sh \
  /home/pnp/Desktop/franka_record_data/put_eraser_into_drawer \
  /home/pnp/Desktop/franka_record_data_10hz/put_eraser_into_drawer_two_views \
  --camera all \
  --source-fps 30 \
  --target-fps 10 \
  --overwrite
```

默认输出目录：

```text
/home/pnp/Desktop/franka_record_data_10hz/<task>
```

## 输出格式

每条 episode 仍然是当前 capture 格式：

```text
0/
  0.pkl.gz
  keyframes.json
  right.mp4
  wrist.mp4
```

`pkl.gz` 顶层结构：

```python
{
    "data": [frame0, frame1, ...],
    "keyframes": [0, ...],
}
```

每帧字段：

```text
schema_version  # 如果源数据存在
pose
joint
gripper_closedness
gripper_01closedness
gripper_width
gripper_target_width
timestamp
right_image
```

如果使用 `--camera all`，还会保留 `wrist_image` 和 `wrist.mp4`。

## 降采样规则

30Hz 到 10Hz 使用固定 stride：

```text
selected_source_indices = [0, 3, 6, 9, ...]
```

也就是说，保留原始数据中每 3 帧的第 1 帧，不做插值。

`timestamp` 保留原始 selected frame 的 unix timestamp，方便追溯真实采集时间。

`keyframes` 从原始索引映射到新索引：

```text
new_keyframe = round(old_keyframe / 3)
```

并去重、裁剪到合法范围，保证至少包含 `0`。

## Metadata

task 输出目录下会额外生成：

```text
downsample_metadata.json
```

里面记录 source/target fps、stride、总 episode 数和每条 episode 的输入/输出帧数。
如果原始 task 中有空目录或缺少 `*.pkl.gz` 的目录，会跳过并记录在 `skipped_without_pkl` 中。
