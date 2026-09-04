# Gen Finger Controller Python SDK

> Pure Python SDK for Gen Finger single-camera devices — camera streaming, tactile sensing, encoder feedback, and distance control without ROS.

[中文](README_CN.md)

[GitHub repository](https://github.com/genrobot-ai/gen_finger_con_python_sdk_release)

License: [MIT License](LICENSE)

## 1 Features

- Pure Python implementation (`FingerSystem`)
- Single-camera image streaming with OpenCV live preview
- Tactile sensor data callbacks (left / right)
- Encoder feedback for finger opening distance
- Finger distance control via `DataBus.set_target_distance()`
- Single-finger and dual-finger CLI launcher (`start_finger.py`)
- Utility scripts for calibration and device ID
- Sine-wave tracking tests and report generation (`--sine-wave`, `--sine-report`)

## 2 Requirements


| Item     | Requirement                              |
| -------- | ---------------------------------------- |
| OS       | Ubuntu 20.04 / 22.04 / 24.04 (recommended) |
| Python   | 3.8+                                     |
| USB      | USB 3.0 port                             |
| Hardware | Gen Finger controller device             |


> A Python virtual environment is recommended to avoid PEP 668 restrictions on Ubuntu 24+.



## 3 Quick Start

> First-time users must complete [USB configuration](docs/usb-setup.md) before running the SDK.

```shell
git clone https://github.com/genrobot-ai/gen_finger_con_python_sdk_release.git
cd gen_finger_con_python_sdk_release
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python3 start_finger.py left
```

Verify feedback and send a control command:

```shell
# Terminal continuously prints finger distance: X.XXX m (encoder feedback)

# Set fixed opening to 5 cm (range: [0.0, 0.2] m)
python3 start_finger.py left --distance 0.05
```

After startup, one camera preview window appears. Press ESC or Ctrl+C to exit.

## 4 Python Interface

Subscribe to sensor data through `FingerSystem` callbacks and publish control commands through `DataBus`.

### 4.1 Data Callbacks


| Callback                            | Type            | Description                          |
| ----------------------------------- | --------------- | ------------------------------------ |
| `capture_frames_callback(camera)`   | `CameraCapture` | Finger camera frames                 |
| `tactile_callback(record_data)`     | `bytes`         | Raw tactile bytes (left / right)     |
| `encoder_callback(record_data)`     | `bytes`         | Finger opening distance feedback (m) |




### 4.2 Control Interface

```python
if self.system.databus:
    self.system.databus.set_target_distance(value)  # value in [0.0, 0.2] (meters)
```



### 4.3 CLI Arguments (`start_finger.py`)


| Argument               | Default     | Description                                              |
| ---------------------- | ----------- | -------------------------------------------------------- |
| `side`                 | —           | Required: `left` or `right`                              |
| `--camera-resolutions` | `1600x1296` | Camera resolution                                        |
| `--no-preview`         | `false`     | Disable OpenCV preview window                            |
| `--camera-fps`         | `60`        | Camera FPS (60 required for ~30 fps)                     |
| `--stream-mode`        | `false`     | Force video stream mode (disable trigger mode; for laptop compatibility) |
| `--distance`           | `0.05`      | Fixed target opening (m); mutually exclusive with `--sine-wave` |
| `--sine-wave`          | `false`     | Enable sinusoidal open/close mode                        |
| `--amplitude`          | `0.025`     | Sine amplitude (m)                                       |
| `--center`             | `0.05`      | Sine center position (m)                                 |
| `--frequency`          | `0.5`       | Sine frequency (Hz)                                      |
| `--duration`           | `10.0`      | Sine duration (s); `0` runs indefinitely                 |
| `--print-tactile-info` | `false`     | Print tactile grid to terminal                           |
| `--sine-report`        | `false`     | Record sine tracking data and generate PNG report (requires `--sine-wave`) |
| `--report-path`        | auto        | Output path for tracking report PNG                      |


Default device paths per side:


| Side    | Serial                 | Camera                      |
| ------- | ---------------------- | --------------------------- |
| `left`  | `/dev/ttyFingerLeft`   | `/dev/finger_camera_left`   |
| `right` | `/dev/ttyFingerRight`  | `/dev/finger_camera_right`  |




## 5 Installation



### 5.1 Install system and Python dependencies

```shell
sudo apt update
sudo apt install -y python3-pip python3-venv python3-full v4l-utils
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

`v4l-utils` provides `v4l2-ctl`, required for USB configuration.

> Before each new terminal session, `cd` to the project directory and run `source venv/bin/activate`. You'll see `(venv)` at the prompt when the environment is active.



### 5.2 Clone the repository

```shell
git clone https://github.com/genrobot-ai/gen_finger_con_python_sdk_release.git
cd gen_finger_con_python_sdk_release
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Main artifacts:


| Artifact           | Path                          |
| ------------------ | ----------------------------- |
| Launcher script    | `start_finger.py`             |
| Core modules       | `scripts/`                    |
| Tactile processing | `tactile_processing.py`       |
| Device utilities   | `scripts/camera_cmd.py`       |
| Calibration output | `scripts/calib_result/`       |




## 6 USB Configuration

Configure udev rules once per USB port before first use. The template is at [config/99-usb-serial.rules](./config/99-usb-serial.rules).

Each finger requires only one serial port and one camera.

Summary:

1. Query serial and camera `KERNELS` values with `udevadm` and `v4l2-ctl`
2. Edit `config/99-usb-serial.rules`
3. Copy to `/etc/udev/rules.d/` and reload rules

For step-by-step instructions with screenshots, see:

- [USB 配置指南 (ZH)](docs/usb-setup_CN.md)
- [USB Configuration Guide (EN)](docs/usb-setup.md)

Default serial symlinks after dual-finger setup: `/dev/ttyFingerLeft`, `/dev/ttyFingerRight`.

Default camera symlinks: `/dev/finger_camera_left`, `/dev/finger_camera_right`.

Verify:

```shell
ls -l /dev/ttyFingerLeft /dev/finger_camera_left
ls -l /dev/ttyFingerRight /dev/finger_camera_right
```



## 7 Usage



### 7.1 Single Finger Demo

```shell
source venv/bin/activate
python3 start_finger.py left
```

Optional arguments:

```shell
python3 start_finger.py left --distance 0.02          # Fixed at 2 cm
python3 start_finger.py left --sine-wave              # Sinusoidal open/close for 10 s
python3 start_finger.py left --no-preview             # Disable preview window
python3 start_finger.py left --camera-fps 60          # 60 required to achieve 30 fps
python3 start_finger.py left --print-tactile-info     # Print tactile grid to terminal
```

After startup, one image window appears. Terminal output:

```
finger distance: X.XXX m    # Finger opening distance feedback
```

Default is `--camera-fps 60`. If frame rate is abnormal on older hardware, try `--camera-fps 30` or add `--stream-mode`.

### 7.2 Dual Finger Demo

Run in two terminals:

```shell
# Terminal A
source venv/bin/activate
python3 start_finger.py left

# Terminal B
source venv/bin/activate
python3 start_finger.py right
```

After startup, two image preview windows appear (one per finger).

For sine-wave tracking tests with report generation, see [测试流程说明.md](测试流程说明.md).

### 7.3 Device Utilities

Do **not** run these while `start_finger.py` or other control processes are active.

**Single device:**

```shell
source venv/bin/activate
python3 scripts/camera_cmd.py camerarc   # Camera calibration (single camera)
python3 scripts/camera_cmd.py MCUID      # Device ID
```

**Dual device (left / right):**

```shell
source venv/bin/activate

python3 scripts/camera_cmd.py left camerarc
python3 scripts/camera_cmd.py left MCUID

python3 scripts/camera_cmd.py right camerarc
python3 scripts/camera_cmd.py right MCUID
```

Finger devices use one camera, so `camerarc` is the normal calibration command. Calibration YAML files are saved to `scripts/calib_result/` (e.g. `cam0_sensor_single.yaml`, `cam0_sensor_left.yaml`).

Override serial port with an environment variable:

```shell
SERIAL_PORT=/dev/ttyFingerLeft python3 scripts/camera_cmd.py MCUID
```



### 7.4 Programming Example

```python
import threading
import time
from scripts import FingerSystem

def encoder_callback(record_data: bytes):
    # Parse encoder data
    pass

def tactile_callback(record_data: bytes):
    # Handle tactile data
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
system.start()  # Blocks until ESC or Ctrl+C
```



## 8 Troubleshooting


| Problem                              | Solution                                                          |
| ------------------------------------ | ----------------------------------------------------------------- |
| Serial port not found                | Run `sudo apt remove brltty`, then replug the device              |
| Camera or serial has wrong path      | Re-check udev rules; see [docs/usb-setup.md](docs/usb-setup.md)   |
| No `finger distance:` output         | Check finger power; test with `sudo minicom -D /dev/ttyFingerLeft -b 921600` |
| Camera won't open                    | Previous process not exited: `pkill -9 -f start_finger.py`, or re-plug USB |
| Low camera frame rate                | Keep `--camera-fps 60`; try `--stream-mode` on older hardware     |
| `Permission denied` on serial/camera | Add user to `dialout` and `video` groups, or `sudo chmod 666 /dev/ttyUSB* /dev/video*` |
| Device utility command fails         | Stop `start_finger.py` and other control processes before running utilities |




## 9 Documentation Index


| Description              | Link                                                       |
| ------------------------ | ---------------------------------------------------------- |
| USB 配置 (ZH)              | [docs/usb-setup_CN.md](docs/usb-setup_CN.md)               |
| USB setup (EN)           | [docs/usb-setup.md](docs/usb-setup.md)                     |
| udev rules template      | [config/99-usb-serial.rules](config/99-usb-serial.rules)   |
| Launcher script          | [start_finger.py](start_finger.py)                         |
| Sine tracking test guide | [测试流程说明.md](测试流程说明.md)                                 |
| Calibration utility      | [scripts/camera_cmd.py](scripts/camera_cmd.py)             |
| Core modules             | [scripts/](scripts/)                                       |

