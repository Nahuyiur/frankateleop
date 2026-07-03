# 点云坐标系统一标定使用说明

这个目录用于把 RealSense RGB-D 点云从相机坐标系统一到 Franka robot base
坐标系。第一版支持固定外部相机的 **eye-to-hand** 标定：

```text
相机固定在外部，例如 middle / left / right
ChArUco 标定板刚性固定在夹爪/末端
手动移动机器人采集多组姿态
求 T_base_camera_color_optical
```

得到外参后，正式采集时仍然只需要保存 RGB-D stream。使用数据时再做：

```text
p_base = T_base_camera @ p_camera
```

这样不同相机、不同时间采集到的点云就能进入同一个 robot base/world frame。

## 0. 坐标约定

本系统统一使用：

```text
T_A_B: 把 B 坐标系下的点变到 A 坐标系
p_A = T_A_B @ p_B
```

固定相机目标是求：

```text
T_base_camera_color_optical
```

当前约定：

- world frame = Franka robot base。
- RealSense/OpenCV color optical frame = `x` 向右，`y` 向下，`z` 向前。
- robot pose 来自 `ee_pose_euler=[x,y,z,roll,pitch,yaw]`。
- `roll/pitch/yaw` 是 radians，等价于 SciPy `Rotation.as_euler("xyz")`。

对固定外部相机，几何关系是：

```text
T_base_gripper_i @ T_gripper_target
=
T_base_camera @ T_camera_target_i
```

OpenCV `solvePnP/ChArUco` 输出的是：

```text
T_camera_target
```

OpenCV `calibrateHandEye` 做 eye-to-hand 时必须喂：

```text
inv(T_base_gripper_i)
T_camera_target_i
```

输出解释为：

```text
T_base_camera
```

这个方向已经通过 synthetic test 验证，不要手动取反。

## 1. 推荐统一入口脚本

从 repo 根目录使用：

```bash
cd /home/pnp/frankateleop
bash 20_pointcloud_calibration.sh --help
```

常用命令：

```bash
# 看完整实操流程
bash 20_pointcloud_calibration.sh workflow

# 检查 OpenCV/aruco/hand-eye 环境
bash 20_pointcloud_calibration.sh doctor

# 列出最近的标定 session
bash 20_pointcloud_calibration.sh latest

# 无硬件自检：编译 + synthetic hand-eye
bash 20_pointcloud_calibration.sh self-test

# 生成 ChArUco 标定板
bash 20_pointcloud_calibration.sh board

# 手动采样
bash 20_pointcloud_calibration.sh capture --camera middle

# 求解 + 自动验证 + doctor
bash 20_pointcloud_calibration.sh solve <session_dir>

# 重新验证已有结果
bash 20_pointcloud_calibration.sh validate <session_dir>

# 把一个 RGB-D episode 的点云变到 robot base frame
bash 20_pointcloud_calibration.sh apply <episode_dir> \
  --camera middle \
  --extrinsic <session_dir>/calibration_result.json

# 把同一帧的多个已标定相机融合到一个 robot base/world-frame PLY
bash 20_pointcloud_calibration.sh fuse <episode_dir> \
  --extrinsic middle=<middle_session>/calibration_result.json \
  --extrinsic left=<left_session>/calibration_result.json \
  --extrinsic right=<right_session>/calibration_result.json
```

便利脚本默认使用 session 目录下的标准文件名：

```text
calibration_result.json
validation_report.json
doctor_report.json
```

不要在 `solve/validate/check` 里传自定义 `--output` 或 `--result`。这些实验参数会绕开统一验收路径；如果确实要做离线实验，请直接调用对应 Python module，并把产物路径写清楚。

## 2. 第一次实操流程

### 2.1 生成并打印标定板

```bash
bash 20_pointcloud_calibration.sh board
```

默认输出：

```text
~/Desktop/franka_record_data/calibration/targets/charuco_7x5_35mm_26mm.png
```

默认参数：

```text
dictionary       = DICT_5X5_250
squares_x        = 7
squares_y        = 5
square_length_m  = 0.035
marker_length_m  = 0.026
```

打印后必须实际测量：

- 一个棋盘格 square 的边长。
- 一个黑色 marker 的边长。

如果打印尺寸和默认值不同，采样时必须传入真实尺寸：

```bash
bash 20_pointcloud_calibration.sh capture \
  --camera middle \
  --square-length-m 0.0348 \
  --marker-length-m 0.0258
```

尺寸错会直接导致外参尺度错，不能靠后处理修正。

### 2.2 固定标定板

把 ChArUco 板刚性固定在夹爪/末端：

- 不要手持。
- 不要贴在软纸上晃动。
- 不要让板相对夹爪滑动。
- 板要尽量平整。

标定算法会同时估计 `T_base_camera` 和 `T_gripper_target`，所以不需要提前知道板相对夹爪的精确位姿，但它必须在整个采集过程中保持不变。

### 2.3 启动手动采样

采样前先确认两件事：

- RealSense 相机能正常打开，目标相机名是 `middle` / `left` / `right` 中的一个。
- robot ZMQ state node 已运行。默认读取 `127.0.0.1:6001`，如果不是这个地址，要显式传：

```bash
bash 20_pointcloud_calibration.sh capture \
  --camera middle \
  --robot-host <host> \
  --robot-port <port>
```

`doctor` 只检查 OpenCV/aruco/hand-eye 软件环境，不会替你证明机器人节点和相机已经连上；这一步必须在现场启动采样时确认。

```bash
bash 20_pointcloud_calibration.sh capture --camera middle
```

窗口中能看到检测 overlay。按键：

```text
s 保存当前 sample
q 退出
```

按 `s` 的条件：

- robot 完全静止。
- overlay 显示 ChArUco valid。
- 标定板不要太靠图像边缘。
- 标定板不要太小或太大。
- 图像不要明显模糊。

按下 `s` 后，脚本会先检查机器人是否稳定，再重新抓取一帧 RGB-D，并在抓帧后再读一次 robot pose 做前后夹紧检查。如果这段时间机器人移动超过约 `1 mm / 0.2 deg`，样本不会保存。

建议采集：

```text
25-40 个姿态
最终 good samples >= 20
```

姿态要覆盖：

- 工作空间左 / 右 / 前 / 后 / 高 / 低。
- 明显 roll / pitch / yaw 变化。
- 不要只平移，旋转多样性很关键。

采样输出类似：

```text
~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS/
  session.json
  samples/
    sample_000000/
      rgb.png
      depth.png
      detection_overlay.png
      metadata.json
```

### 2.4 求解和验证

```bash
bash 20_pointcloud_calibration.sh solve \
  ~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS
```

默认少于 `20` 个 good samples 会直接失败，不会写正式外参。调试时可以直接调用 Python module 并显式使用 `--allow-low-samples`，但这种结果不能用于正式采集。

输出：

```text
calibration_result.json
validation_report.json
doctor_report.json
```

最重要的是：

```text
calibration_result.json 里的 T_base_camera
validation_report.json 里的 pass
doctor_report.json 里的 status / warnings / errors
```

只有 `validation_report pass=true` 且 `doctor` 没有 errors 时，才考虑用于正式采集。

`validate <session_dir>` 默认复用 `calibration_result.json` 里的 `used_sample_ids`，也就是重新检查求解最终采用的样本集合。诊断 outlier 影响时可以运行：

```bash
bash 20_pointcloud_calibration.sh validate <session_dir> --all-accepted
```

这会用所有 gate-accepted samples 重新计算残差，默认写到 `validation_all_accepted_report.json`，不会覆盖正式的 `validation_report.json`。它可能比正式报告更严格；如果它失败但默认 validate 通过，通常说明 outlier 确实存在，需要看 overlay 和 `excluded_outlier_ids`。

## 3. 如何判断标定没有问题

不要只看点云“差不多对齐”。至少看下面几层。

### 3.1 Sample gate

每个 sample 的检测质量要过关：

```text
detected ChArUco corners >= 12，推荐 >= 16
reprojection RMS < 2.0 px，推荐中位数 < 1.5 px
reprojection max error < 5.0 px
board bbox area 在图像面积 3% - 60% 左右
corner 距图像边缘 >= 8 px
如果有 depth-plane 检查：board ROI depth valid ratio >= 70%
```

如果大量样本被 gate reject，常见原因：

- 标定板太远或太斜。
- 图像模糊。
- 打印尺寸参数不对。
- 板靠近图像边缘。
- 光照反光。

### 3.2 姿态多样性

`doctor_report.json` 里有：

```text
pose_diversity.translation_span_norm_m
pose_diversity.rotation_span_deg
```

建议：

```text
translation span > 0.20 m
rotation span > 25 deg
```

只在一个小区域里轻微移动，可能也能算出外参，但不可信。

### 3.3 Hand-eye 残差

求出 `T_base_camera` 后，每个样本会反算：

```text
T_gripper_target_i =
inv(T_base_gripper_i) @ T_base_camera @ T_camera_target_i
```

如果标定正确，所有 `T_gripper_target_i` 应该几乎一样，因为标定板固定在夹爪上。

验收目标：

```text
translation residual median < 10 mm
translation residual p90    < 20 mm
rotation residual median    < 1.5 deg
rotation residual p90       < 3 deg
```

如果超过这个范围：

- 先看 outlier overlay。
- 重新采更多旋转更丰富的姿态。
- 检查 `square_length_m / marker_length_m`。
- 确认相机没有移动。
- 确认 robot node 的 pose 是当前真实末端位姿。

### 3.4 World-frame 点云验证

标定通过后，先录一个短 RGB-D smoke episode：

```bash
bash 18_record_rgb_pointclouds.sh \
  --task rgb_pointcloud_calib_smoke \
  --camera-names middle \
  --depth-cameras middle \
  --auto-start \
  --duration-sec 3
```

如果现场没有顶层 `18_record_rgb_pointclouds.sh` wrapper，可以用等价 Python 入口：

```bash
python -m pointcloud.record_rgb_pointclouds \
  --task rgb_pointcloud_calib_smoke \
  --camera-names middle \
  --depth-cameras middle \
  --auto-start \
  --duration-sec 3
```

再应用外参：

```bash
bash 20_pointcloud_calibration.sh apply \
  ~/Desktop/franka_record_data/rgb_pointcloud_calib_smoke/0 \
  --camera middle \
  --extrinsic ~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json \
  --frame middle \
  --stride 4
```

`apply` 会检查外参文件里的 `camera_name` 是否和 `--camera` 一致。多相机时不要把 middle 的外参套到 left/right；除非做调试，否则不要使用 `--allow-camera-mismatch`。

如果已经分别标定了多个固定相机，可以直接融合到一个 robot base/world frame：

```bash
bash 20_pointcloud_calibration.sh fuse \
  ~/Desktop/franka_record_data/rgb_pointcloud_calib_smoke/0 \
  --extrinsic middle=~/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json \
  --extrinsic left=~/Desktop/franka_record_data/calibration/left_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json \
  --extrinsic right=~/Desktop/franka_record_data/calibration/right_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json \
  --frame middle \
  --stride 4 \
  --voxel-size-m 0.006
```

`fuse` 默认要求每个外参目录下有通过的 `validation_report.json`，并拒绝 `provisional=true` 的低样本结果。也就是说，先分别完成：

```bash
bash 20_pointcloud_calibration.sh solve <middle_session>
bash 20_pointcloud_calibration.sh solve <left_session>
bash 20_pointcloud_calibration.sh solve <right_session>
```

再融合。

`fuse` 做的事情是：

```text
每个相机: RGB-D + intrinsics -> camera optical frame point cloud
每个相机: T_base_camera @ p_camera -> robot base/world frame
所有相机: concat -> voxel downsample -> multi_camera_frameXXXXXX_base_cloud.ply
```

默认输出：

```text
<episode>/world_pointcloud/
  middle_frameXXXXXX_base_cloud.ply
  left_frameXXXXXX_base_cloud.ply
  right_frameXXXXXX_base_cloud.ply
  multi_camera_frameXXXXXX_base_cloud.ply
  multi_camera_frameXXXXXX_base_summary.json
```

`fuse` 默认只融合你传了 `--extrinsic camera=...` 的相机。也可以显式指定：

```bash
--camera-names middle,left,right
```

如果想用一个文件管理多相机外参，可以写：

```json
{
  "schema_version": "frankateleop_world_extrinsics_v1",
  "world_frame": "robot_base",
  "camera_frame": "camera_color_optical",
  "cameras": {
    "middle": {
      "calibration_result": "/home/pnp/Desktop/franka_record_data/calibration/middle_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json"
    },
    "left": {
      "calibration_result": "/home/pnp/Desktop/franka_record_data/calibration/left_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json"
    },
    "right": {
      "calibration_result": "/home/pnp/Desktop/franka_record_data/calibration/right_eye_to_hand_YYYYMMDD_HHMMSS/calibration_result.json"
    }
  }
}
```

然后运行：

```bash
bash 20_pointcloud_calibration.sh fuse \
  ~/Desktop/franka_record_data/rgb_pointcloud_calib_smoke/0 \
  --extrinsics-map ~/Desktop/franka_record_data/calibration/world_extrinsics.json \
  --frame middle
```

外参文件里如果有 `camera_name` / `serial_number`，脚本会和 episode metadata 做匹配检查。正常不要使用：

```bash
--allow-camera-mismatch
--allow-metadata-mismatch
--allow-unvalidated-extrinsics
```

这些选项只适合临时调试。

输出在：

```text
<episode>/world_pointcloud/
```

检查点云：

- 桌面应该在 robot base 下有稳定高度，不应该竖起来。
- 相机 frustum 的位置应符合真实安装位置。
- 标定板/夹爪附近点云应和机器人末端运动一致。
- 多相机时，同一桌面/物体区域不应出现明显双层。

### 3.5 什么时候必须重标定

以下情况必须重新标定或至少重新验证：

- 相机或支架被碰过。
- 相机 serial 更换。
- 分辨率、color/depth alignment 模式改变。
- 标定板固定方式改变。
- robot node / 末端 frame 定义改变。
- 正式采集前 smoke test 的 world-frame 点云明显错位。

## 4. 常见错误和症状

### 点云整体跑到奇怪方向

大概率是矩阵方向取反或轴系误解。不要手动翻 camera y/z；RealSense/OpenCV optical frame 本来就是 `x right, y down, z forward`，`T_base_camera` 会负责变到 base frame。

### 残差几十厘米

常见原因：

- 传错 session。
- 标定板尺寸单位错，把 mm 当 m 或反过来。
- robot pose 不是当前真实 pose。
- 标定板相对夹爪移动了。
- 相机在采样过程中被碰了。

### reprojection 很好但 world 点云不对

reprojection 只证明视觉检测内部一致，不证明 robot-camera 外参正确。继续看 hand-eye residual、pose diversity、world-frame smoke test。

### 多相机融合有双层

每个相机分别检查：

```text
T_base_middle
T_base_left
T_base_right
```

不要用 ICP 修正后再宣称标定正确。ICP 只能做诊断：如果需要 ICP 修很多，说明外参还不够好。

## 5. 文件说明

```text
targets.py              生成 ChArUco 标定板
capture_samples.py      手动采样，不控制机器人，只读取 robot pose
detect_charuco.py       检测 ChArUco 并求 T_camera_target
solve_eye_to_hand.py    求 T_base_camera，自动写 validation_report
validate_extrinsic.py   重新验证已有结果
doctor.py               环境/session/result 诊断
apply_extrinsics.py     把 RGB-D episode 点云变到 robot base frame
fuse_world_pointclouds.py 多相机 world-frame 点云融合
synthetic_handeye_test.py  无硬件数学自检
synthetic_fusion_test.py   无硬件多相机融合自检
geometry.py             4x4 transform 和 ee_pose_euler 转换
```

## 6. 最小验收 checklist

正式采集前，每个固定外部相机至少满足：

```text
[ ] ChArUco 板刚性固定，尺寸已测量并写入命令
[ ] good samples >= 20，推荐 25-40
[ ] 姿态覆盖足够，有明显旋转变化
[ ] reprojection RMS median < 1.5 px，p90 < 2.0 px
[ ] hand-eye translation median < 10 mm，p90 < 20 mm
[ ] hand-eye rotation median < 1.5 deg，p90 < 3 deg
[ ] outliers < 20%
[ ] doctor 无 errors
[ ] world-frame smoke pointcloud 视觉上合理
[ ] 多相机 `fuse` 后没有明显双层、错位、镜像或相机套错外参
[ ] 相机没有在标定后被移动
```

满足这些条件，才把 `calibration_result.json` 用到正式采集数据处理中。
