# GELLO EEF teleoperation test

This is an additive test path.  The existing joint teleoperation scripts are
unchanged.

The adapter reads the calibrated 7-DoF GELLO state, evaluates Franka forward
kinematics, anchors the leader and follower poses at startup, and sends bounded
absolute Cartesian poses through the existing `command_ee_pose` robot-node
method.  Startup therefore has zero pose discontinuity even if the GELLO and
Franka joint configurations differ.

## Start the left-arm test with one command

From the repository root, first validate the complete path without starting
any hardware service:

```bash
cd /home/pnp/frankateleop
bash A_run_left_gello_eef.sh --check
```

Run monitor-only mode first.  This starts the left robot server, gripper
server, EEF robot node, and GELLO adapter, but the adapter does not send motion
commands:

```bash
cd /home/pnp/frankateleop
bash A_run_left_gello_eef.sh
```

After checking the displayed deltas, clear the workspace, keep a hand on the
emergency stop, and explicitly enable motion with conservative scaling:

```bash
cd /home/pnp/frankateleop
bash A_run_left_gello_eef.sh --execute \
  --translation-scale 0.5 --rotation-scale 0.5 \
  --max-anchor-translation 0.12 --max-anchor-rotation 0.6
```

Press Ctrl-C once to stop the adapter and every stack process started by the
launcher.  Logs for each component are saved under `logs/gello_eef/`.

The launcher refuses to kill or replace an existing service on ports 50052,
50054, or 6002.  Stop any existing joint/EEF stack before using it.

## Manual four-terminal alternative

Use four terminals, as with the existing 1-4 stack:

```bash
cd /home/pnp/frankateleop
bash left_franka/1_launch_robot.sh
```

```bash
cd /home/pnp/frankateleop
bash left_franka/2_launch_gripper.sh
```

```bash
cd /home/pnp/frankateleop
bash left_franka/3_launch_eef_node.sh
```

First run the adapter without motion and move the GELLO gently while checking
the printed target deltas:

```bash
cd /home/pnp/frankateleop
bash left_franka/4_run_eef_gello.sh
```

Stop it with Ctrl-C, clear the robot workspace, keep a hand on the emergency
stop, then explicitly enable commands:

```bash
bash left_franka/4_run_eef_gello.sh --execute
```

Useful first-test limits:

```bash
bash left_franka/4_run_eef_gello.sh --execute \
  --translation-scale 0.5 --rotation-scale 0.5 \
  --max-anchor-translation 0.12 --max-anchor-rotation 0.6
```

`--check-only` validates the ports, EE node mode, GELLO, FK model, and anchor,
then exits without commanding motion.  The live adapter stops on encoder jumps,
workspace violations, anchor-envelope violations, tracking errors, node errors,
or Ctrl-C.  Ctrl-C leaves the Cartesian controller holding its final target.

Do not run the existing `4_run_env.sh` at the same time; both programs would
otherwise command the same robot node.
