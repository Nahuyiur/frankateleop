# Franka GUI Capture

这个模块是 `franka_capture` 的桌面 GUI 入口，用 PyQt6 实现。它不替代原来的
`0_record_fr3_pipeline.sh` 和 `6_record_fr3.sh`；原脚本仍然可以照常使用。

## 启动

```bash
bash A_run_left_arm_capture_gui.sh
```

无硬件测试界面：

```bash
bash A_run_left_arm_capture_gui.sh --mock
```

右臂单臂 GUI：

```bash
bash B_run_right_arm_capture_gui.sh
```

双臂 GUI：

```bash
bash C_run_dual_arm_capture_gui.sh
```

`14_run_capture_gui.sh` 仅作为旧入口兼容，会转发到 A 单臂 GUI。

默认保存到：

```text
/home/pnp/Desktop/franka_record_data/<task>/<episode_index>/
```

GUI 采集模式固定使用这个根目录。A 左臂单臂 GUI 固定使用
`left_wrist/left/middle` 三路相机；B 右臂单臂 GUI 固定使用
`middle/right/right_wrist` 三路相机；C 双臂 GUI 使用全部配置相机。
新增任务时只需要输入任务名称，GUI 会创建：

```text
/home/pnp/Desktop/franka_record_data/<task>/
```

GUI 采集频率固定为 `30 Hz`。界面不提供 FPS 切换，启动脚本也不接受 FPS 覆盖。

任务名变化后，中间面板会显示 `下一条路径`，用于确认下一条数据会保存到哪个
`task/index`。

## 使用流程

1. 打开 GUI 后，右侧会启动相机预览。
2. 左侧点击启动按钮：A 为 `启动 1-4`，B 为 `启动右臂 1-4`，
   C 为 `启动双臂 1-4`。GUI 会按对应模式启动机器人、夹爪、robot node 和 teleop env。
   如果终端提示 sudo 密码，需要回到启动 GUI 的那个终端输入一次；脚本 1 启动
   Franka realtime server 时需要 sudo 权限。

   如果本机存在下面这个私有文件，GUI 会自动用它预缓存 sudo 权限，不再手动输入：

   ```text
   ~/.franka_gui_sudo_password
   ```

   这个文件只应该保存在本机，权限建议为 `600`。也可以用环境变量
   `FRANKA_GUI_SUDO_PASSWORD_FILE` 指向其他私有文件。
3. 在中间选择任务名，或点击 `新增采集任务` 输入一个任务名称。
4. 可选填写 `附加 metadata`。普通文本会保存为 `user_metadata.note`；JSON object
   会原样保存到 `metadata.json` 的 `user_metadata` 字段下。
5. 按 `s` 或点击 `开始` 开始录制。
6. 按 `e` 或点击 `保存当前`，当前 episode 会进入后台保存队列；相机预览会继续刷新。
7. 保存还没结束时，可以继续按 `s` 录下一条数据。

快捷键：

```text
s: 开始/继续
w: 暂停
e: 保存当前 episode
d: 丢弃当前 episode
k: 添加关键帧
q: 保存当前 episode
```

## 输出格式

GUI 保存的数据保持和当前 `franka_capture` 兼容：

```text
/home/pnp/Desktop/franka_record_data/<task>/<index>/
  <index>.pkl.gz
  keyframes.json
  metadata.json
  preview_all.mp4
  <camera>.mp4
```

单臂 `pkl.gz` 里每帧包含：

```text
schema_version
pose
joint
gripper_closedness
gripper_01closedness
gripper_width
gripper_target_width
timestamp
<camera_name>_image
```

双臂 GUI 使用 `franka_dual_v2`，每帧保存 `left_*` 和 `right_*`
机器人字段，并保存全部 `<camera_name>_image`。

`preview_all.mp4` 是 GUI 额外保存的低分辨率横向拼接预览视频，用于快速回看；
它不改变 `pkl.gz` 中的逐帧数据。

`gripper_closedness` 为连续闭合度：`0=open, 1=closed`。
`gripper_01closedness` 为二值闭合状态：`closedness >= 0.5 -> 1`。
`gripper_width` 为反馈实际开口宽度，单位米。
`gripper_target_width` 为命令目标开口宽度，单位米。

图像字段仍然是 OpenCV BGR 顺序，因此现有 HDF5 和 LeRobot 转换脚本可以继续读取。
