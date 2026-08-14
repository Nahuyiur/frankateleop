# DROID delta-EEF replay

This package replays canonical DROID `panda_link8` body-frame increments. It
anchors the relative path at the FR3's live end-effector pose; it never sends
the source robot's absolute world pose.

Selected episode: `88685`, a successful 107-frame (7.07 s) kettle-button
trajectory. Physical replay defaults to half speed, first returns to the DROID
episode's frame-0 arm joint position, opens to the episode's initial gripper
width, replays both delta EEF and gripper targets, and returns to the configured
left-arm home joints after successful completion.

```bash
cd /home/muka/frankateleop
./droid_test_replay/start_ee_stack.sh

# Live-pose preflight; sends no commands.
./droid_test_replay/run_replay.sh

# Only after checking the predicted pose range and clearing the workspace.
./droid_test_replay/run_replay.sh --execute --confirm-episode 88685
```

Safety checks cover the source-start approach, joint/FK consistency, control
mode, every SE(3) increment, total path, maximum
anchor displacement, predicted Cartesian workspace, Cartesian-controller health
and the final return-home approach. `start_ee_stack.sh` recovers a stopped Cartesian policy
at the current pose without returning the arm to configured initial joints.
Ctrl-C stops new commands; the Cartesian impedance controller holds its latest
target.
