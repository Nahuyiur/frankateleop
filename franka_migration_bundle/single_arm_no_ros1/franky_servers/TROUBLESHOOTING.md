# Franky 服务器问题解决方案

## 问题总结

1. **服务器崩溃**: 在重新配置时服务器会崩溃
2. **连接拒绝**: 客户端无法连接到端口 5000
3. **Reset 失败**: 因为服务器不可用

## 解决方案

### 方案 1: 手动启动服务器（推荐用于调试）

```bash
# 进入服务器目录
cd /home/franka/Code/FR3/franky_servers

# 使用正确的 Python 环境启动
/home/franka/miniconda3/bin/python server.py
```

### 方案 2: 使用启动脚本

```bash
# 启动服务器
/home/franka/Code/FR3/franky_servers/start_server.sh

# 查看日志
tail -f /tmp/franky_server.log
```

### 方案 3: 使用 Watchdog（推荐用于生产）

Watchdog 会自动监控服务器并在崩溃时重启：

```bash
# 启动 watchdog（后台运行）
nohup /home/franka/Code/FR3/franky_servers/franky_watchdog.sh > /dev/null 2>&1 &

# 查看 watchdog 日志
tail -f /tmp/franky_watchdog.log

# 停止 watchdog
pkill -f franky_watchdog.sh
```

## 当前服务器状态

服务器已启动但未连接到机器人。客户端会在初始化时自动连接。

检查状态：
```bash
curl http://127.0.0.1:5000/health | python -m json.tool
```

## 关于 Reset 问题

### 为什么 Reset 仍然可能失败

即使服务器运行，reset 可能因为以下原因失败：

1. **模式切换延迟**: 从 async 切换到 sync 需要时间
2. **机器人状态**: 机器人可能处于错误状态
3. **网络延迟**: HTTP 请求超时

### 改进建议

#### 选项 A: 增加模式切换等待时间

编辑 `franka_controller_franky.py`，在 `reset_joint()` 和 `reset_cartesian()` 中：

```python
# 从
time.sleep(0.1)
# 改为
time.sleep(0.5)  # 增加到 500ms
```

#### 选项 B: 完全使用同步模式（牺牲遥操作流畅度）

在 `franka_env.py` 初始化 controller 时：

```python
self._controller = FrankaControllerFranky(
    robot_ip=self.config.robot_ip,
    relative_dynamics_factor=0.015,
    use_async_motion=False,  # 添加这一行
)
```

#### 选项 C: 分离遥操作和训练环境（推荐）

- 遥操作/演示收集: 使用 `use_async_motion=True`
- 自动训练: 使用 `use_async_motion=False`

## 防止服务器崩溃

### 问题根源

服务器在重新连接时可能崩溃，特别是当：
1. 机器人已经连接
2. 尝试用不同参数重新连接
3. 机器人处于运动状态

### 解决方法

**不要在运行时重新配置服务器**。而是：

1. 在启动训练前确保服务器运行
2. 让客户端在初始化时连接（只连接一次）
3. 如果需要改变参数，重启整个系统

## 快速检查清单

运行训练前：

```bash
# 1. 检查服务器是否运行
curl http://127.0.0.1:5000/health

# 2. 如果没有运行，启动它
cd /home/franka/Code/FR3/franky_servers
/home/franka/miniconda3/bin/python server.py &

# 3. 等待 2 秒
sleep 2

# 4. 再次检查
curl http://127.0.0.1:5000/health

# 5. 启动训练
cd /home/franka/Code/wangyinxi/RLinf/examples/embodiment
./run_fr3_peginsertion_async.sh
```

## 监控服务器

实时查看服务器日志：

```bash
tail -f /tmp/franky_server.log
```

查看最近的错误：

```bash
grep -i error /tmp/franky_server.log | tail -20
```

## 紧急恢复

如果一切都失败了：

```bash
# 1. 杀死所有相关进程
pkill -f "server.py"
pkill -f "franky_watchdog"
pkill -f "train_async.py"

# 2. 清理 PID 文件
rm -f /tmp/franky_server.pid

# 3. 重启服务器
cd /home/franka/Code/FR3/franky_servers
/home/franka/miniconda3/bin/python server.py > /tmp/franky_server.log 2>&1 &

# 4. 等待并验证
sleep 3
curl http://127.0.0.1:5000/health

# 5. 重启训练
cd /home/franka/Code/wangyinxi/RLinf/examples/embodiment
./run_fr3_peginsertion_async.sh
```
