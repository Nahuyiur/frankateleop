#!/usr/bin/env python3
"""Startup script — launches finger hardware and cameras."""

import sys
import os
import argparse
import struct
import time
import cv2
import math
import threading
import numpy as np
from typing import List, Optional, Tuple

_sdk_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _sdk_root not in sys.path:
    sys.path.insert(0, _sdk_root)
from scripts import FingerSystem
from tactile_processing import (
    convert_tactile_448_to_1000,
    set_tactile_grid_print_enabled,
    set_tactile_grid_print_max_hz,
    submit_tactile_1000_grid_print,
)

_sine_report_enabled = False
_sine_report_side = "left"
_sine_report_path = ""
_tracking_lock = threading.Lock()
_target_samples: List[Tuple[float, float]] = []   # (elapsed_s, target_m)
_encoder_samples: List[Tuple[float, float]] = []  # (elapsed_s, encoder_m)
_tracking_start_time = 0.0
_tracking_active = False
_current_target = 0.0
_TRACKING_MATCH_THRESH = 0.005  # 5mm: encoder must be within this of target to start tracking
_skipped_encoder_count = 0


def capture_frames_callback(camera):
    """Callback for camera frame capture — grab + display on the same thread."""
    if camera.show_preview:
        for cam in camera.cameras:
            cv2.namedWindow(cam['window_name'], cv2.WINDOW_NORMAL)
            cv2.resizeWindow(cam['window_name'], 640, 480)

    frame_interval = 1.0 / camera.target_fps

    try:
        while camera.running:
            start_time = time.monotonic()
            frames_data = []

            for cam in camera.cameras:
                ret = cam['cap'].grab()
                frame = None
                if ret:
                    ret2, frame = cam['cap'].retrieve()
                if frame is not None:
                    cam['frame_count'] += 1
                    now = time.monotonic()
                    cam['disp_fps_ts'].append(now)
                    if len(cam['disp_fps_ts']) > 30:
                        cam['disp_fps_ts'] = cam['disp_fps_ts'][-30:]
                    if len(cam['disp_fps_ts']) >= 2:
                        dt = cam['disp_fps_ts'][-1] - cam['disp_fps_ts'][0]
                        if dt > 0:
                            cam['disp_fps_val'] = (len(cam['disp_fps_ts']) - 1) / dt
                    cam['cap_fps_val'] = cam['disp_fps_val']

                frames_data.append((cam, frame))

            if camera.show_preview:
                _display_frames(camera, frames_data)

            elapsed = time.monotonic() - start_time
            sleep_time = max(0, frame_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    except Exception as e:
        print(f"Capture error: {e}")
    finally:
        for cam in camera.cameras:
            try:
                cam['cap'].release()
            except:
                pass
        if camera.show_preview:
            cv2.destroyAllWindows()


def _display_frames(camera, frames_data):
    """Show camera preview windows with FPS overlay."""
    for cam, frame in frames_data:
        if frame is not None:
            timestamp = time.strftime("%H:%M:%S", time.localtime())
            info_text = f"Camera_{cam['id']} | {timestamp} | Frames: {cam['frame_count']}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0), 2)
            fps_text = f"Cap: {cam['cap_fps_val']:.1f}  Disp: {cam['disp_fps_val']:.1f}"
            cv2.putText(frame, fps_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)
            cv2.imshow(cam['window_name'], frame)
    if cv2.waitKey(1) == 27:
        camera.running = False


def tactile_callback(record_data: bytes):
    """Tactile callback: convert and enqueue grid print (non-blocking for serial parse thread)."""
    try:
        left_tactile_500, right_tactile_500 = convert_tactile_448_to_1000(record_data)
        submit_tactile_1000_grid_print(left_tactile_500 + right_tactile_500)
    except Exception as e:
        print(f"Tactile data handler error: {e}")


def encoder_callback(record_data: bytes):
    """Encoder data callback."""
    global _tracking_active, _skipped_encoder_count
    try:
        encoder_value = struct.unpack(">f", record_data)[0]
        if _sine_report_enabled and _tracking_start_time > 0:
            if not _tracking_active:
                if abs(encoder_value - _current_target) <= _TRACKING_MATCH_THRESH:
                    _tracking_active = True
                    print(f"[TRACKING] activated: encoder={encoder_value:.4f} m, "
                          f"target={_current_target:.4f} m, skipped {_skipped_encoder_count} samples")
                else:
                    _skipped_encoder_count += 1
                    return
            elapsed = time.time() - _tracking_start_time
            with _tracking_lock:
                _encoder_samples.append((elapsed, encoder_value))
        else:
            print(f"finger distance: {encoder_value:.3f} m")
    except Exception as e:
        print(f"Encoder data handler error: {e}")


class SineWaveController:
    """Sinusoidal finger position control."""
    
    def __init__(self, system: FingerSystem, amplitude: float = 0.05,
                 center: float = 0.05, frequency: float = 0.5, duration: float = 1000,
                 auto_exit: bool = False):
        self.system = system
        self.amplitude = amplitude
        self.center = center
        self.frequency = frequency
        self.duration = duration
        self.running = False
        self.control_thread = None
        self.start_time = 0
        self.control_interval = 1.0 / 30.0
        self._auto_exit = auto_exit
        
    def start(self):
        """Start sinusoidal control."""
        if self.running:
            return
        if self.amplitude <= 0 or self.center - self.amplitude < 0 or self.center + self.amplitude > 0.2:
            print(" Sine wave parameters out of valid range")
            return
        
        self.running = True
        self.start_time = time.time()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        print(f"🚀 Sine wave started: center={self.center:.3f}m, amplitude=±{self.amplitude:.3f}m, freq={self.frequency:.2f}Hz")
    
    def stop(self):
        """Stop sinusoidal control."""
        if not self.running:
            return
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
    
    def _control_loop(self):
        """Control loop thread body."""
        global _tracking_start_time, _current_target
        try:
            if _sine_report_enabled:
                _tracking_start_time = self.start_time
            while self.running:
                cycle_start = time.time()
                current_time = time.time() - self.start_time

                if self.duration > 0 and current_time >= self.duration:
                    self.running = False
                    break

                value = self.center + self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)
                value = max(0.0, min(0.2, value))

                if _sine_report_enabled:
                    _current_target = value
                    if _tracking_active:
                        with _tracking_lock:
                            _target_samples.append((current_time, value))

                if self.system.databus:
                    self.system.databus.set_target_distance(value)

                elapsed = time.time() - cycle_start
                sleep_time = max(0, self.control_interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except Exception as e:
            print(f" Sine wave control error: {e}")
            self.running = False
        finally:
            if self._auto_exit and self.system.camera:
                self.system.camera.running = False


class FingerController:
    """High-level finger control (fixed distance vs sine wave)."""
    
    def __init__(self, system: FingerSystem):
        self.system = system
        self.sine_wave_controller: Optional[SineWaveController] = None
        
    def set_fixed_distance(self, distance: float):
        """Set a fixed finger opening distance."""
        if distance < 0.0 or distance > 0.2:
            print(f" Warning: distance {distance} out of range [0.0, 0.2], ignored")
            return
        
        if self.sine_wave_controller and self.sine_wave_controller.running:
            self.sine_wave_controller.stop()
        
        try:
            self.system.set_finger_distance(distance)
            print(f"Fixed finger distance set: {distance} m ({distance*100:.1f} cm)")
        except Exception as e:
            print(f" Failed to set finger distance: {e}")
    
    def start_sine_wave(self, amplitude: float = 0.05, center: float = 0.05,
                        frequency: float = 0.5, duration: float = 60.0,
                        auto_exit: bool = False):
        """Start sinusoidal control."""
        if self.sine_wave_controller and self.sine_wave_controller.running:
            self.sine_wave_controller.stop()

        self.sine_wave_controller = SineWaveController(
            system=self.system, amplitude=amplitude, center=center,
            frequency=frequency, duration=duration, auto_exit=auto_exit,
        )
        self.sine_wave_controller.start()
    
    def stop_sine_wave(self):
        """Stop sinusoidal control."""
        if self.sine_wave_controller:
            self.sine_wave_controller.stop()
    
    def is_sine_wave_running(self) -> bool:
        """Return True if sine wave control is active."""
        return self.sine_wave_controller.running if self.sine_wave_controller else False


def generate_sine_report(side: str, report_path: str):
    """Generate tracking report: terminal stats + PNG plot."""
    with _tracking_lock:
        targets = list(_target_samples)
        encoders = list(_encoder_samples)

    if len(targets) < 2 or len(encoders) < 2:
        print("[TRACKING] Not enough data to generate report")
        return

    enc_duration = encoders[-1][0] - encoders[0][0]
    enc_freq = (len(encoders) - 1) / enc_duration if enc_duration > 0 else 0
    print(f"[ENCODER] STM32 return frequency: {enc_freq:.2f} Hz "
          f"(samples={len(encoders)}, duration={enc_duration:.2f}s)")

    t_target = np.array([s[0] for s in targets])
    v_target = np.array([s[1] for s in targets])
    t_encoder = np.array([s[0] for s in encoders])
    v_encoder = np.array([s[1] for s in encoders])

    v_encoder_interp = np.interp(t_target, t_encoder, v_encoder)

    t_eval = t_target
    tgt_eval = v_target
    enc_eval = v_encoder_interp

    if len(t_eval) < 2:
        print("[TRACKING] Not enough data after alignment")
        return

    duration = t_eval[-1] - t_eval[0]
    print(f"[TRACKING] samples={len(t_eval)}, duration={duration:.2f}s, mode=sine, side={side}")
    if _skipped_encoder_count > 0:
        print(f"[TRACKING] waited for encoder to reach target position, "
              f"skipped {_skipped_encoder_count} encoder samples "
              f"(threshold={_TRACKING_MATCH_THRESH*1000:.1f} mm)")

    errors = enc_eval - tgt_eval
    abs_errors = np.abs(errors)
    mae = np.mean(abs_errors)
    rmse = np.sqrt(np.mean(errors ** 2))
    max_abs = np.max(abs_errors)
    mean_signed = np.mean(errors)
    print(f"[TRACKING] error: MAE={mae:.6f} m ({mae*1000:.2f} mm), "
          f"RMSE={rmse:.6f} m ({rmse*1000:.2f} mm), "
          f"MaxAbs={max_abs:.6f} m ({max_abs*1000:.2f} mm), "
          f"MeanSigned={mean_signed:.6f} m ({mean_signed*1000:.2f} mm)")

    # latency estimation via cross-correlation
    if len(tgt_eval) > 10:
        dt = np.mean(np.diff(t_eval))
        tgt_norm = tgt_eval - np.mean(tgt_eval)
        enc_norm = enc_eval - np.mean(enc_eval)
        corr_full = np.correlate(enc_norm, tgt_norm, mode='full')
        max_idx = np.argmax(corr_full)
        lag_samples = max_idx - (len(tgt_norm) - 1)
        latency_s = lag_samples * dt
        peak_corr = corr_full[max_idx] / (np.linalg.norm(tgt_norm) * np.linalg.norm(enc_norm)) if (np.linalg.norm(tgt_norm) * np.linalg.norm(enc_norm)) > 0 else 0
        print(f"[TRACKING] latency: estimated={latency_s:.3f} s ({latency_s*1000:.1f} ms), "
              f"corr={peak_corr:.3f} (positive means encoder lags target)")

    # generate PNG
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("[TRACKING] matplotlib not installed. Install with: pip install matplotlib")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True,
                                    gridspec_kw={'height_ratios': [2, 1]})

    ax1.plot(t_target, v_target * 1000, label='target distance', color='tab:blue', linewidth=1)
    ax1.plot(t_target, v_encoder_interp * 1000, label='encoder distance', color='tab:orange', linewidth=1)
    ax1.set_ylabel('distance (mm)')
    ax1.set_title(f'Sine Tracking Report — {side}')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    ax2.plot(t_target, np.abs(v_encoder_interp - v_target) * 1000, color='tab:red', linewidth=1)
    ax2.set_ylabel('abs error (mm)')
    ax2.set_xlabel('time (s)')
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    if not report_path:
        ts = time.strftime("%Y%m%d_%H%M%S")
        report_path = f"{side}_sine_tracking_{ts}.png"
    plt.savefig(report_path, dpi=150)
    plt.close(fig)
    print(f"[TRACKING] report saved: {report_path}")


def main():
    """CLI entry point."""
    SIDE_CONFIG = {
        'left': {
            'serial_port': "/dev/ttyFingerLeft",
            'video_devices': ["/dev/finger_camera_left"],
        },
        'right': {
            'serial_port': "/dev/ttyFingerRight",
            'video_devices': ["/dev/finger_camera_right"],
        },
    }
    
    parser = argparse.ArgumentParser(description="Start finger system (optional sine wave mode)")
    parser.add_argument("side", type=str, choices=['left', 'right'],
                       help="Finger side: left or right")
    parser.add_argument("--camera-resolutions", type=str, default="1600x1296",
                       help="Camera resolution as 'widthxheight'")
    parser.add_argument("--no-preview", action="store_true",
                       help="Do not show camera preview windows")
    parser.add_argument("--camera-fps", type=int, default=60,
                       help="Target camera display frame rate (default 60, trigger mode needs 60 to achieve 30fps)")
    parser.add_argument("--stream-mode", action="store_true",
                       help="Force camera to video stream mode (disable trigger mode, for laptop compatibility)")

    control_group = parser.add_mutually_exclusive_group()
    control_group.add_argument("--distance", type=float, default=None,
                              help="Fixed finger distance in meters, range [0.0, 0.2]")
    control_group.add_argument("--sine-wave", action="store_true",
                              help="Enable sine wave control mode")
    
    parser.add_argument("--amplitude", type=float, default=0.025,
                       help="Sine amplitude in meters (default 0.025)")
    parser.add_argument("--center", type=float, default=0.05,
                       help="Sine center position in meters (default 0.05)")
    parser.add_argument("--frequency", type=float, default=0.5,
                       help="Sine frequency in Hz (default 0.5)")
    parser.add_argument("--duration", type=float, default=10.0,
                       help="Sine duration in seconds; 0 = run forever (default 10.0)")
    parser.add_argument(
        "--print-tactile-info",
        action="store_true",
        help="Print tactile grid to terminal (50 lines: L10 + gap + R10 per line); default is off",
    )
    parser.add_argument(
        "--tactile-print-hz",
        type=float,
        default=0.0,
        help="Cap tactile grid print rate (Hz); 0 = no cap. Reduces terminal load while showing latest frame per print.",
    )
    parser.add_argument(
        "--sine-report",
        action="store_true",
        help="Record target/encoder data during sine wave and generate tracking report PNG",
    )
    parser.add_argument(
        "--report-path",
        type=str,
        default=None,
        help="Output path for tracking report PNG (default: timestamped file in current dir)",
    )
    
    args = parser.parse_args()

    if args.sine_report and not args.sine_wave:
        parser.error("--sine-report requires --sine-wave")

    global _sine_report_enabled, _sine_report_side, _sine_report_path
    if args.sine_report:
        _sine_report_enabled = True
        _sine_report_side = args.side
        _sine_report_path = args.report_path or ""

    set_tactile_grid_print_enabled(args.print_tactile_info)
    set_tactile_grid_print_max_hz(args.tactile_print_hz)
    config = SIDE_CONFIG[args.side]
    
    system = FingerSystem(
        serial_port=config['serial_port'],
        camera_resolutions=args.camera_resolutions,
        show_preview=not args.no_preview,
        video_devices=config['video_devices'],
        tactile_callback=tactile_callback,
        encoder_callback=encoder_callback,
        capture_frames_callback=capture_frames_callback,
        camera_fps=args.camera_fps,
        trigger_mode=not args.stream_mode,
    )
    
    controller = FingerController(system)
    
    def setup_control_mode():
        """Apply control mode after DataBus is ready."""
        max_wait_time = 10.0
        wait_interval = 0.1
        elapsed_time = 0.0
        
        while elapsed_time < max_wait_time:
            if system.databus is not None:
                time.sleep(0.5)
                if args.sine_wave:
                    controller.start_sine_wave(
                        amplitude=args.amplitude, center=args.center,
                        frequency=args.frequency, duration=args.duration,
                        auto_exit=args.sine_report,
                    )
                elif args.distance is not None:
                    controller.set_fixed_distance(args.distance)
                else:
                    controller.set_fixed_distance(0.05)
                return
            time.sleep(wait_interval)
            elapsed_time += wait_interval
        print(" Warning: system init timed out; control mode not applied")
    
    threading.Thread(target=setup_control_mode, daemon=True).start()
    
    try:
        system.start()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if controller.is_sine_wave_running():
            controller.stop_sine_wave()
        system.stop()
        if _sine_report_enabled:
            generate_sine_report(_sine_report_side, _sine_report_path)


if __name__ == "__main__":
    main()
