# GELLO Dynamixel damping tuner

This tool edits and applies a separate JSON profile. It does not change PID
constants in `run_env.py` when a slider moves.

## Safe workflow

1. Stop `run_env.py` and any other process that owns the GELLO serial port.
2. Keep the Franka disabled/yellow during leader tuning.
3. Start the left or right tuner:

   ```bash
   bash T_run_dynamixel_damping_tuner.sh --side left
   bash T_run_dynamixel_damping_tuner.sh --side right
   ```

4. Click **Connect (torque OFF)**. The tuner reads model numbers, device current
   limits, gains, current settings, and the present pose. Unsupported motor
   models are rejected.
5. Click **CAPTURE CURRENT GOALS** before enabling any joint. Select only one
   joint for the first test and start with a low current.
6. Move sliders, then click **APPLY SAFELY**. Slider movement alone never writes
   the bus. Apply is rejected if an enabled goal is more than 0.20 rad from the
   measured pose.
7. Use **Too weak** / **Too strong** in the binary-search panel to narrow one
   parameter at a time. Every applied setting and binary decision is stored in
   the profile history.
8. Click **TORQUE OFF** before disconnecting. Closing the tuner also attempts to
   disable all seven arm motors.

The default profile paths are:

- `config/dynamixel_damping_left.json`
- `config/dynamixel_damping_right.json`

The tuner is intentionally standalone and owns the serial port while open. It
does not modify or restart `run_env.py`; stop the tuner before starting normal
teleoperation. The saved JSON is the experiment record and can be reopened by
the tuner without changing source constants.

Validate a profile without opening hardware or a GUI:

```bash
bash T_run_dynamixel_damping_tuner.sh \
  --profile config/dynamixel_damping_left.json \
  --validate-only
```

The profile and driver use milliamps for XC330 model numbers 1190/1200. The
driver also verifies every requested current against each device's Current
Limit register before applying it.
