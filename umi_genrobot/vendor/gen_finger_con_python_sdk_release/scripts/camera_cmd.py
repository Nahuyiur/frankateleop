#!/usr/bin/env python3
"""
Camera Command Tool - Pure Python implementation
Replaces camera_cmd.sh for sending camera calibration commands
"""

import sys
import time
import os

# Supports running as script: python camera_cmd.py or python -m ...scripts.camera_cmd
if __name__ == "__main__" or not __package__:
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _parent_dir = os.path.dirname(_script_dir)
    if _parent_dir not in sys.path:
        sys.path.insert(0, _parent_dir)
    from scripts.databus import DataBus, find_serial_port, find_finger_serial_by_side
    from scripts.pack import CmdPack
else:
    from .databus import DataBus, find_serial_port, find_finger_serial_by_side
    from .pack import CmdPack


VALID_COMMANDS = ['1234', 'camerarc', 'MCUID']


def is_camera_calib_command(command: str) -> bool:
    return command.startswith('camera')


def camera_calib_callback(camera_pack):
    """Callback when camera calibration payload is received."""
    print("Camera calibration data received")


def main():
    """CLI entry point."""
    usage = """
Usage:
  Single-device mode (default when left/right omitted):
    python3 scripts/camera_cmd.py {1234|camerarc|MCUID}
  Dual-device mode (left or right):
    python3 scripts/camera_cmd.py {left|right} {1234|camerarc|MCUID}

  Optional env: SERIAL_PORT=/dev/ttyUSB0 (overrides left/right default port)

  Arguments:
    left/right - Optional finger side (omit for single-device mode)
    1234       - Confirm calibration complete
    camerarc   - Calibrate center camera (writes cam0_sensor_{single|left|right}.yaml)
    MCUID      - Query device MCUID
    """

    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(usage)
        sys.exit(0 if len(sys.argv) >= 2 else 1)

    if len(sys.argv) == 2:
        side = 'single'
        record_value = sys.argv[1]
    else:
        side = sys.argv[1].lower()
        if side not in ('left', 'right'):
            print(f"Error: first argument must be left or right, got '{sys.argv[1]}'")
            print(usage)
            sys.exit(1)
        record_value = sys.argv[2]

    if record_value not in VALID_COMMANDS:
        print(f"Error: unsupported command: {record_value}")
        print(usage)
        sys.exit(1)

    yaml_filename = ""
    if record_value == "camerarc":
        yaml_filename = f"cam0_sensor_{side}.yaml"

    result_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_result")
    if yaml_filename:
        os.environ['CALIB_YAML_FILENAME'] = yaml_filename
        print(f"Will write YAML: {yaml_filename}")
        print(f"Save path: {os.path.join(result_dir, yaml_filename)}")
    elif 'CALIB_YAML_FILENAME' in os.environ:
        del os.environ['CALIB_YAML_FILENAME']

    serial_port = os.environ.get('SERIAL_PORT', '')
    if not serial_port:
        if side in ('left', 'right'):
            serial_port = find_finger_serial_by_side(side)
        else:
            serial_port = find_serial_port("ttyUSB")

    if not serial_port:
        print(" No configured serial port found")
        sys.exit(1)

    print(f"Side: {side}")
    print(f"Serial port: {serial_port}")
    print(f"Sending camera calibration command: {record_value}")

    try:
        bus = DataBus(
            tty_port=serial_port,
            baudrate=921600,
            is_calib_cmd=True,
            camera_calib_callback=camera_calib_callback,
        )
        time.sleep(1.0)
        bus.send_camera_calib_cmd(record_value)

        if record_value == 'MCUID':
            bus.wait_for_calib_response(3.0)
        elif is_camera_calib_command(record_value):
            bus.wait_for_calib_response(2.0)
        else:
            time.sleep(0.5)

        bus.stop()

        if record_value == "1234":
            print("Calibration OK !")
        elif record_value == "MCUID":
            print("MCUID query executed")
        else:
            print(f"Finished sending command: {record_value}")
            if yaml_filename:
                yaml_path = os.path.join(result_dir, yaml_filename)
                if os.path.exists(yaml_path):
                    print(f"YAML file created: {yaml_path}")
                else:
                    print(" YAML file not found; check device response")
                    sys.exit(1)
    except Exception as e:
        print(f" Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
