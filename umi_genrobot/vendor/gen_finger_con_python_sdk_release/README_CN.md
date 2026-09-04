# Gen Finger Controller Python SDK

> 用于 Gen Finger 单相机设备的纯 Python SDK，不依赖 ROS，支持相机图像、触觉传感、编码器反馈及开合距离控制。

[English](README.md)

[GitHub代码](https://github.com/genrobot-ai/gen_finger_con_python_sdk_release)

License: [MIT License](LICENSE)

## 1 功能特性

- 纯 Python 实现（`FingerSystem`）
- 单相机图像流，支持 OpenCV 实时预览
- 触觉数据回调（左 / 右）
- finger 开合距离编码器反馈
- 通过 `DataBus.set_target_distance()` 控制 finger 开合
- 单指 / 双指 CLI 启动脚本（`start_finger.py`）
- 标定、设备 ID 等工具脚本
- 正弦波跟踪测试与报告生成（`--sine-wave`、`--sine-report`）

## 2 环境要求


| 项目     | 要求                               |
| ------ | -------------------------------- |
| 系统     | Ubuntu 20.04 / 22.04 / 24.04（推荐） |
| Python | 3.8+                             |
| USB    | USB 3.0 接口                       |
| 硬件     | Gen Finger controller 设备         |


> 建议使用 Python 虚拟环境，避免 Ubuntu 24+ 的 PEP 668 限制。



## 3 快速开始

> 首次使用请先完成 [USB 配置](docs/usb-setup_CN.md)。

```shell
git clone https://github.com/genrobot-ai/gen_finger_con_python_sdk_release.git
cd gen_finger_con_python_sdk_release
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 start_finger.py left
```

验证反馈并发送控制指令：

```shell
# 终端输出持续打印 finger distance: X.XXX m（编码器反馈）

# 固定开合 5 cm（范围 [0.0, 0.2] m）
python3 start_finger.py left --distance 0.05
```

启动后会弹出一个相机预览窗口。按 ESC 或 Ctrl+C 退出。

## 4 Python 接口

通过 `FingerSystem` 回调获取传感器数据，通过 `DataBus` 发布控制指令。

### 4.1 数据回调


| 回调                                | 数据类型            | 说明               |
| --------------------------------- | --------------- | ---------------- |
| `capture_frames_callback(camera)` | `CameraCapture` | finger 相机帧       |
| `tactile_callback(record_data)`   | `bytes`         | 左侧 / 右侧触觉原始字节    |
| `encoder_callback(record_data)`   | `bytes`         | finger 开合距离反馈（m） |




### 4.2 控制接口

```python
if self.system.databus:
    self.system.databus.set_target_distance(value)  # value ∈ [0.0, 0.2] (米)
```



### 4.3 命令行参数（`start_finger.py`）


| 参数                     | 默认值         | 说明                                    |
| ---------------------- | ----------- | ------------------------------------- |
| `side`                 | —           | 必填，`left` 或 `right`                   |
| `--camera-resolutions` | `1600x1296` | 相机分辨率                                 |
| `--no-preview`         | `false`     | 关闭 OpenCV 预览窗口                        |
| `--camera-fps`         | `60`        | 相机帧率（60 以获得约 30 fps）                  |
| `--stream-mode`        | `false`     | 强制视频流模式（关闭触发模式，兼容部分笔记本）               |
| `--distance`           | `0.05`      | 固定目标开合距离（m），与 `--sine-wave` 互斥        |
| `--sine-wave`          | `false`     | 启用正弦波开合模式                             |
| `--amplitude`          | `0.025`     | 正弦波振幅（m）                              |
| `--center`             | `0.05`      | 正弦波中心位置（m）                            |
| `--frequency`          | `0.5`       | 正弦波频率（Hz）                             |
| `--duration`           | `10.0`      | 正弦波持续时间（s），`0` 表示持续运行                 |
| `--print-tactile-info` | `false`     | 终端打印触觉网格                              |
| `--sine-report`        | `false`     | 记录正弦跟踪数据并生成 PNG 报告（需配合 `--sine-wave`） |
| `--report-path`        | 自动生成        | 跟踪报告 PNG 输出路径                         |


单指默认设备路径：


| 侧别      | 串口                    | 相机                         |
| ------- | --------------------- | -------------------------- |
| `left`  | `/dev/ttyFingerLeft`  | `/dev/finger_camera_left`  |
| `right` | `/dev/ttyFingerRight` | `/dev/finger_camera_right` |




## 5 安装



### 5.1 安装系统与 Python 依赖

```shell
sudo apt update
sudo apt install -y python3-pip python3-venv python3-full v4l-utils
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

`v4l-utils` 提供 USB 配置所需的 `v4l2-ctl` 命令。

> 每次新开终端使用前，需先 `cd` 到项目目录并执行 `source venv/bin/activate`。命令行前出现 `(venv)` 表示环境已激活。



### 5.2 拉取仓库

```shell
git clone https://github.com/genrobot-ai/gen_finger_con_python_sdk_release.git
cd gen_finger_con_python_sdk_release
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

主要产物：


| 产物     | 路径                      |
| ------ | ----------------------- |
| 启动脚本   | `start_finger.py`       |
| 核心模块   | `scripts/`              |
| 触觉处理   | `tactile_processing.py` |
| 设备工具   | `scripts/camera_cmd.py` |
| 标定结果目录 | `scripts/calib_result/` |




## 6 USB 配置

首次使用前需为每个 USB 口配置 udev 规则，模板见 [config/99-usb-serial.rules](./config/99-usb-serial.rules)。

每只 finger 只需 1 个串口 + 1 个相机

简要步骤：

1. 用 `udevadm` 和 `v4l2-ctl` 查询串口与相机的 `KERNELS` 值
2. 编辑 `config/99-usb-serial.rules`
3. 复制到 `/etc/udev/rules.d/` 并 reload

详细图文步骤见：

- [USB 配置指南 (ZH)](docs/usb-setup_CN.md)
- [USB Configuration Guide (EN)](docs/usb-setup.md)

双指配置后的默认串口软链接：`/dev/ttyFingerLeft`、`/dev/ttyFingerRight`。

默认相机软链接：`/dev/finger_camera_left`、`/dev/finger_camera_right`。

验证：

```shell
ls -l /dev/ttyFingerLeft /dev/finger_camera_left
ls -l /dev/ttyFingerRight /dev/finger_camera_right
```



## 7 使用方法



### 7.1 单指 Demo

```shell
source venv/bin/activate
python3 start_finger.py left
```

可选参数：

```shell
python3 start_finger.py left --distance 0.02            # 固定开 2 cm
python3 start_finger.py left --sine-wave                # 正弦持续开合 10 s
python3 start_finger.py left --no-preview               # 关闭预览窗口
python3 start_finger.py left --camera-fps 60            # 设备需 60 才能跑到 30 fps
python3 start_finger.py left --print-tactile-info       # 终端打印触觉网格
```

启动后弹出一个图像窗口，终端持续输出：

```
finger distance: X.XXX m    # finger 开合距离反馈
```

默认 `--camera-fps 60`。若旧设备帧率异常，可改为 `--camera-fps 30` 或加 `--stream-mode`。

### 7.2 双指 Demo

在两个终端分别运行：

```shell
# 终端 A
source venv/bin/activate
python3 start_finger.py left

# 终端 B
source venv/bin/activate
python3 start_finger.py right
```

启动后弹出两个图像预览窗口（每指一个）。

正弦波跟踪测试（含报告生成）见 [测试流程说明.md](测试流程说明.md)。

### 7.3 设备工具命令

运行设备工具前**不要**同时启动 `start_finger.py` 或其他控制进程。

**单设备：**

```shell
source venv/bin/activate
python3 scripts/camera_cmd.py camerarc   # 相机标定（单相机）
python3 scripts/camera_cmd.py MCUID      # 设备 ID
```

**双设备（左 / 右）：**

```shell
source venv/bin/activate

python3 scripts/camera_cmd.py left camerarc
python3 scripts/camera_cmd.py left MCUID

python3 scripts/camera_cmd.py right camerarc
python3 scripts/camera_cmd.py right MCUID
```

finger 新设备只有 1 个相机，常用标定命令为 `camerarc`。标定 YAML 文件保存至 `scripts/calib_result/`（如 `cam0_sensor_single.yaml`、`cam0_sensor_left.yaml`）。

通过环境变量指定串口：

```shell
SERIAL_PORT=/dev/ttyFingerLeft python3 scripts/camera_cmd.py MCUID
```



### 7.4 编程示例

```python
import threading
import time
from scripts import FingerSystem

def encoder_callback(record_data: bytes):
    # 解析编码器数据
    pass

def tactile_callback(record_data: bytes):
    # 处理触觉数据
    pass

system = FingerSystem(
    serial_port="/dev/ttyFingerLeft",
    video_devices=["/dev/finger_camera_left"],
    encoder_callback=encoder_callback,
    tactile_callback=tactile_callback,
    camera_fps=60,
)

def apply_control():
    while system.databus is None:
        time.sleep(0.1)
    time.sleep(0.5)
    system.set_finger_distance(0.05)  # 5 cm

threading.Thread(target=apply_control, daemon=True).start()
system.start()  # 阻塞运行，ESC 或 Ctrl+C 退出
```



## 8 常见问题


| 问题                      | 解决方法                                                                    |
| ----------------------- | ----------------------------------------------------------------------- |
| 找不到串口                   | 执行 `sudo apt remove brltty`，重新插拔设备                                      |
| 相机或串口路径不对               | 检查 udev 规则，见 [docs/usb-setup_CN.md](docs/usb-setup_CN.md)               |
| 无 `finger distance:` 输出 | 检查夹爪电源；用 `sudo minicom -D /dev/ttyFingerLeft -b 921600` 验证串口            |
| 相机打不开                   | 上一进程未退出：`pkill -9 -f start_finger.py`，或重新插拔 USB                         |
| 相机帧率偏低                  | 保持 `--camera-fps 60`；旧设备可试 `--stream-mode`                              |
| `Permission denied`     | 将用户加入 `dialout`、`video` 组，或临时 `sudo chmod 666 /dev/ttyUSB* /dev/video*` |
| 设备工具命令失败                | 运行工具前先停止 `start_finger.py` 及其他控制进程                                      |




## 9 文档索引


| 说明             | 链接                                                       |
| -------------- | -------------------------------------------------------- |
| USB 配置 (ZH)    | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)             |
| USB setup (EN) | [docs/usb-setup.md](docs/usb-setup.md)                   |
| udev 规则模板      | [config/99-usb-serial.rules](config/99-usb-serial.rules) |
| 启动脚本           | [start_finger.py](start_finger.py)                       |
| 正弦跟踪测试流程       | [测试流程说明.md](测试流程说明.md)                                   |
| 标定工具           | [scripts/camera_cmd.py](scripts/camera_cmd.py)           |
| 核心模块           | [scripts/](scripts/)                                     |


