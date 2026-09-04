# 采集数据复核

数据复核工具读取一个当前或历史 episode，不发送机器人命令。支持单臂/双臂 v2（PKL
内嵌图像）和 v3（相机 MP4）格式。

```bash
bash D_review_capture_data.sh \
  /home/pnp/Desktop/Muka_NAS/<task>/High_Quality/<index>
```

也可以不传路径，在窗口中选择 episode 目录、`metadata.json` 或 `.pkl.gz`。

页面内容：

- 按 action frame index 同步播放所有声明的相机视角；声明视角缺失或视频帧数不足时拒绝打开。
- 单臂显示一组 Joint；双臂可切换 left/right，两组轨迹都参与质量事件检查。
- 每个 Joint 独立显示角度时间轴和 Franka 物理关节限位黄虚线。
- 单帧变化 `>0.10 rad` 为 WARN，`>0.20 rad` 为 FAIL；速度 `>6 rad/s` 为 FAIL。
- 显示 EEF 的 XY/XZ 轨迹、夹爪反馈/目标宽度、关键帧和可跳转的质量事件列表。

这些阈值与当前 V validator 的动作跳变检查一致；物理关节限位是额外的可视诊断。顶部
`V 核验` 和诊断表复用完整单条 V validator，包括相机集合、视频帧数、v3 contract、
时间间隙、机器人读取和夹爪检查；Joint 事件不会冒充完整验证结论。

v3 回看按需在后台读取视频帧，只保留最新请求，不会把完整多路视频加载进内存。v2 会在
同目录 MP4 存在时优先用 MP4 并释放 PKL 图像引用；所有 schema 的 PKL 在压缩文件大于
256 MiB 或逐块解压计数超过 1 GiB 时默认拒绝直接打开，避免错误 metadata 或 gzip 大小
回绕绕过内存保护。确认内存足够时可显式设置
`FRANKA_REVIEW_ALLOW_LARGE_V2=1`。播放器使用相邻 action timestamp 调度，而不是假设
每帧间隔恒定；当前 schema 没有逐相机硬件曝光时间戳，因此只能证明保存后的 frame-index
对齐，不能声称曝光级硬同步。
