# 双臂 π0.5 推理与动作执行

执行入口已安装在 **98**：`/home/pnp/frankateleop/dual_pi05/run_policy.py`。

保留当前机器人启动窗口。先松开 Pico 手柄、让两臂静止，再在 98 的另一个终端运行：

```bash
/home/pnp/miniconda3/envs/franka_capture/bin/python \
  /home/pnp/frankateleop/dual_pi05/run_policy.py \
  --execute --pause-pico \
  --prompt "Stack the cubes into a tower" \
  --cycles 1 --steps-per-chunk 5 --speed-scale 0.2
```

这是一次短动作试运行：程序从当前姿态开始，使用两侧真实相机和状态调用 106，再把左右各前 5 个目标插值并成对下发。左臂通过 98 本机 6002 端口，右臂通过 86 的 172.16.0.3:6001。它不会自动执行 home。

确认短动作符合预期后，可将 `--cycles 1` 改为 `--cycles 30`，进行连续的“观测 → 推理 → 执行前 5 步 → 再观测”。默认速度缩放为 0.2，同时限制平移、旋转和夹爪速度。两臂网络下发是软件配对，不是硬件级原子同步。

`--pause-pico` 仅暂停已核实身份的 Pico 控制 Python 进程，保留机器人节点和它的启动窗口。**不要关闭原来的 Pico 启动窗口**：该窗口的退出清理会停止双臂底层服务。程序结束后 Pico 保持暂停，避免两套控制器同时发送动作。

按 **Ctrl+C** 中止：程序停止后续策略动作，尝试给每个可达机械臂发送当前实测末端姿态作为保持目标。网络故障时保持命令可能无法送达；真机急停仍需现场可用。

需要恢复 Pico 时，先松开两只手柄的离合/抓握键，再在 98 执行：

```bash
/home/pnp/miniconda3/envs/franka_capture/bin/python \
  /home/pnp/frankateleop/dual_pi05/run_policy.py --resume-pico
```

只做推理和执行计划预览，去掉 `--execute --pause-pico`：

```bash
/home/pnp/miniconda3/envs/franka_capture/bin/python \
  /home/pnp/frankateleop/dual_pi05/run_policy.py \
  --prompt "Stack the cubes into a tower"
```

结果保存在 98 `/home/pnp/frankateleop/dual_pi05/runtime/policy_execution.json`。被暂停的 Pico 进程身份记录在同目录 `paused_pico.json`。完整实现与配置也保存在 106 `/home/ubuntu/frankapolicy/openpi/dual_franka/`。

执行前会检查双臂处于 ee 模式、工具配置一致、关节距离软限位足够、机器人静止、当前图像与状态足够新、两臂目标数量一致、目标位姿有限、工作空间和 TCP 间距，以及是否有其他控制器插入动作。一次推理的每个目标都使用该次观测的固定 TCP 锚点。重规划时从上一条已发送目标开始插值，避免跟踪延迟造成目标突跳。

任一机械臂返回错误、HOLD 或超时都会中止执行前缀；失败的运动命令不会重试。夹爪观测用实测宽度和驱动的 0.2m 归一化范围，执行宽度限制在当前 Pico 的 0.09m 范围，限制次数会写入记录。

当前配置使用工具局部 z 偏移 0.20196m，与最后检查到的运行中 Pico 参数一致。98 和 106 的 `hardware_profile.json` 指纹必须匹配；Pico 参数变化时会拒绝执行。Home 参考仍是用户指定的 `config/initial_joints_v0.5.json`，与 Pico 当前可能使用的其他 home 文件分别记录。

已通过 106 上的 55 项变换和执行器测试，以及 98 实际 Python 环境中的 22 项执行器测试。这些测试使用模拟机器人接口；本次部署没有执行真机运动。相机投影、TCP 方向、完整机械臂碰撞规划和实际叠积木成功率仍未现场验证。
