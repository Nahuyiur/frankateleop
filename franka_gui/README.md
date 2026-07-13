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

A/B/C 三个 GUI 会分别记住上次填写的任务名、`Text instruction` 和
`附加 metadata`，下一次打开同一个 GUI 时自动恢复。这个状态只用于恢复界面输入，
不会写进 episode 数据目录，也不会改变 `metadata.json`、`pkl.gz` 或视频格式。
默认状态文件位于：

```text
~/.local/state/frankateleop/franka_gui/last_inputs_<profile>.json
```

其中 A/B/C 分别使用 `left`、`right`、`dual` profile。直接运行
`python -m franka_gui.app` 时，默认按 `--mode` 分开保存；也可以用
`--profile-key <name>` 指定独立 profile。如果手动覆盖 A/B/C 脚本里的
`--mode`，建议同时覆盖 `--profile-key`，避免模式和草稿 profile 不一致。

默认保存到：

```text
/home/pnp/Desktop/Muka_NAS/<task>/<quality>/<episode_index>/
```

A/B/C 默认直接在 NAS 的隐藏 staging 目录录制：

```text
/home/pnp/Desktop/Muka_NAS/.recording/<uuid>/episode/
```

视频队列仍有固定上限，并在独立后台线程编码，因此不会随着 episode 时长无限占用
内存。按 `h/l/f` 后，轻量 action/state trajectory 和 metadata 会写入同一 staging
目录；所有文件写完后，目录会原子 rename 成最终 `<task>/<quality>/<index>/`。因此
NAS 上只会看到完整 episode，不会看到半写入的数字目录。NAS 未挂载时 GUI 会拒绝开始
录制，避免误写进本地同名目录。

A 左臂单臂 GUI 固定使用 `left_wrist/left/middle` 三路相机；B 右臂单臂 GUI 固定使用
`middle/right/right_wrist` 三路相机；C 双臂 GUI 使用全部配置相机。新增任务时只需要
输入任务名称；实际 NAS 目录会在第一次保存该任务数据时创建：

```text
/home/pnp/Desktop/Muka_NAS/<task>/
```

如需使用旧的本地缓存加延迟同步模式，可在启动前设置
`FRANKA_GUI_STORAGE_MODE=local-outbox`。该模式会使用
`/home/pnp/Desktop/franka_record_cache/outbox/<uuid>/` 和 `franka_sync`；它不是
A/B/C 的默认模式。

GUI 采集频率固定为 `30 Hz`。界面不提供 FPS 切换，启动脚本也不接受 FPS 覆盖。
GUI 画面只拉取最新帧并以约 `15 Hz` 刷新，旧预览帧不会排队；这只降低界面绘制
开销，不改变写入相机 MP4 和 action 的 `30 Hz` 采集频率。

任一路配置相机缺失或在 episode 中掉线时，GUI 会拒绝开始或暂停当前 episode；
保存时只保留掉线前相机和 action 都完整的前缀，绝不会静默缩减该 episode 的相机集合。

任务名变化后，中间面板会显示 `下一条路径`，用于确认下一条数据会保存到哪个
`task/quality/index`。当前质量目录为 `High_Quality`、`Low_Quality` 和
`Failure`，三个目录各自独立从 `0` 开始编号。

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
4. 填写 `Text instruction`，这项不能为空，会保存到 `metadata.json` 和
   `instruction.txt`。可选填写 `附加 metadata`。普通文本会保存为 `user_metadata.note`；JSON object
   会原样保存到 `metadata.json` 的 `user_metadata` 字段下。
5. 按 `s` 或点击 `开始` 开始录制。
6. 按 `e`、`q` 或点击 `保存当前` 后，当前 episode 会进入蓝色 `JUDGING`
   状态，等待质量分层。
7. 在 `JUDGING` 状态下，按 `h` 保存到 `High_Quality`，按 `l` 保存到
   `Low_Quality`，按 `f` 保存到 `Failure`。`Failure` 用于值得保留的失败
   轨迹；明显误操作、无训练价值的数据仍然按 `d` 丢弃。
8. 按 `h/l/f` 后，episode 会进入后台 NAS staging 保存队列；相机预览会继续刷新。
   完整写入后会原子发布到最终目录，可以立刻修改 instruction 并按 `s` 录下一条数据。
   关闭 GUI 会等待进行中的保存安全收尾。

左侧 `Replay` 按钮用于从 GUI 启动已有 replay 脚本。选择 episode 目录、
`metadata.json` 或 `.pkl.gz` 后，GUI 会先检查数据类型是否匹配当前窗口：
A 只接受左臂单臂数据，B 只接受右臂单臂数据，C 只接受双臂数据。检查通过后，
单臂会调用 `7_replay_fr3.sh`，双臂会调用 `16_replay_bi_arm_pipeline.sh`。
默认是硬件 dry-run；它会连接 robot node 并检查当前起点，但不会发送运动命令。
如果起点偏差超过普通阈值、但仍在自动靠近安全上限内，dry-run 会提示执行模式将先
慢速靠近 frame 0。`只检查文件` 模式只做 episode preflight，不启动 robot node、
SSH 或硬件栈。选择执行 replay 时会再次弹出确认。Replay 运行期间，采集和机器人栈
控制入口会被锁住；日志窗口可以隐藏，再次点击 `Replay` 会重新打开当前日志。
左臂执行 replay 前 GUI 会先停止本机 `4_run_env`，保留 1-3 robot/node 栈；
双臂会先停止当前 GUI 双臂栈，再交给 `16_replay_bi_arm_pipeline.sh` 重新启动
replay 需要的 1-3。右臂 execute 如果当前右臂 GUI 1-4 栈仍在运行，会被拦截，
避免远端 `4_run_env` 和 replay 同时控制右臂；文件检查和硬件 dry-run 不受影响。

快捷键：

```text
s: 开始/继续
w: 暂停
e: 结束当前 episode，进入 JUDGING
h: 保存到 High_Quality
l: 保存到 Low_Quality
f: 保存到 Failure
d: 丢弃当前 episode，不写入任何质量目录
k: 录制中添加关键帧；非录制时保存当前所有相机帧
q: 结束当前 episode，进入 JUDGING
```

左侧 `保存帧` 按钮与非录制状态下的 `k` 等价。它会要求当前 GUI 配置的每一路
相机都已有预览画面，并将原始分辨率 RGB 图像异步保存到：

```text
/home/pnp/Desktop/Muka_NAS/keyframe/<YYYYMMDD_HHMMSS_microseconds>/
  <camera_name>.png
```

该目录不属于 task/episode 数据集，不影响录制、质量分层、replay 或 validator。
写入过程先使用隐藏临时目录，所有 PNG 完成后才原子发布最终时间戳目录；保存失败不会
保留不完整的最终截图目录。录制中 `k` 保持原有关键帧标记语义，`保存帧` 按钮会禁用。

## 输出格式

GUI 保存的数据保持和当前 `franka_capture` 兼容：

```text
/home/pnp/Desktop/Muka_NAS/<task>/<quality>/<index>/
  <index>.pkl.gz
  instruction.txt
  keyframes.json
  metadata.json
  preview_all.mp4
  <camera>.mp4
```

`metadata.json` 中会写入 `quality`、`text_instruction` 和
`relative_episode_dir`，其中 `quality` 为 `High_Quality`、`Low_Quality`
或 `Failure`。

GUI 新采集使用 `franka_single_v3` / `franka_dual_v3`。`pkl.gz` 只保存逐帧
action、state、timestamp 和 `frame_index`，不再重复保存 RGB 数组；图像由同目录
相机 MP4 按 frame index 对齐。单臂每帧包含：

```text
schema_version
pose
joint
gripper_closedness
gripper_01closedness
gripper_width
gripper_target_width
timestamp
```

双臂每帧保存 `left_*` 和 `right_*` 机器人字段。旧版 v2 episode 不修改；replay 和
validator 支持单/双臂 v2/v3。现有单臂 LeRobot/HDF5 转换和 downsample 会继续读取
旧版内嵌图像，也能从新版 MP4 恢复旧消费者需要的 BGR 图像；它们原本就不负责双臂转换。

相机 MP4 仍保存相机输入的完整分辨率，当前配置是 `640x480 @ 30 FPS`，没有在
采集链路中缩放。`metadata.json.image_storage.cameras` 会记录每路视频的宽、高、
通道数和帧数，validator 会逐项核对。

`preview_all.mp4` 是 GUI 额外保存的低分辨率横向拼接预览视频，用于快速回看；
它不改变 `pkl.gz` 中的逐帧数据。

`gripper_closedness` 为连续闭合度：`0=open, 1=closed`。
`gripper_01closedness` 为二值闭合状态：`closedness >= 0.5 -> 1`。
`gripper_width` 为反馈实际开口宽度，单位米。
`gripper_target_width` 为命令目标开口宽度，单位米。

图像字段仍然是 OpenCV BGR 顺序，因此现有 HDF5 和 LeRobot 转换脚本可以继续读取。
