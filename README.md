# gello_crx

GELLO leader-arm teleoperation for the FANUC CRX-10iA/L. Plain `rclpy`, three
nodes, no ZMQ and no GELLO runtime dependency.

```
gello_leader --/gello_leader/leader_states--> gello_teleop --> /forward_position_controller/commands
     |                                              ^
     +--------/gello_leader/trigger--> gello_gripper|
                                                    |
                              /joint_states --------+
```

## Quick start (Linux)

```bash
# 1. Clone into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/SungJBae/CRX_GELLO.git gello_crx

# 2. Build
cd ~/ros2_ws
colcon build --packages-select gello_crx

# 3. Source the workspace
source install/setup.bash

# 4. Verify
ros2 pkg list | grep gello_crx
```

Then follow **Step 1** (calibrate) and **Step 2** (mock test) below.

> **Pulling updates later:** when you or a teammate push changes to GitHub,
> pull them on your Linux laptop with:
> ```bash
> cd ~/ros2_ws/src/gello_crx
> git pull
> cd ~/ros2_ws && colcon build --packages-select gello_crx && source install/setup.bash
> ```

## Why the guards live here

The FANUC hardware interface forwards commands verbatim. No velocity clamp, no
discontinuity check, no delta limit. A hand on a leader arm is a non-smooth
input by definition, and FANUC documents that collaborative speed clamping can
still be exceeded by jittery trajectories. The low-pass filter, the per-tick
rate limiter and the engage-time sync ramp are the only things between the
leader and the servos.

Enable is a **heartbeat, not a latch**. If the enable topic goes quiet for
0.5 s the node drops to IDLE. Killing the runner, closing the terminal or
losing the network all stop motion without anyone sending a stop.

## Step 0 -- One-time setup

### Install dependencies

```bash
# Dynamixel SDK
pip install --index-url https://elilillyco.jfrog.io/artifactory/api/pypi/pypi/simple \
    dynamixel-sdk

# Serial port access
sudo usermod -aG dialout $USER    # log out and back in afterwards
```

### Clone into your ROS 2 workspace

A ROS 2 workspace is just a directory with a `src/` folder. `colcon build`
looks inside `src/` for packages (any folder with a `package.xml`).

```bash
# Create a workspace if you do not have one
mkdir -p ~/ros2_ws/src

# Clone the package
cd ~/ros2_ws/src
git clone https://github.com/SungJBae/CRX_GELLO.git gello_crx

# Build and source
cd ~/ros2_ws
colcon build --packages-select gello_crx
source install/setup.bash
```

Confirm the leader is on the bus:

```bash
ls -l /dev/ttyUSB*
```

### Flash servos to high baudrate (one-time)

The XL330 servos default to 57 600 baud, which is too slow for 100 Hz reading
of 7 servos (~20 ms per cycle, exceeding the 10 ms tick). Flash them to
2 Mbps:

```bash
ros2 run gello_crx gello_set_baudrate \
    --port /dev/ttyUSB0 \
    --current-baud 57600 \
    --target-baud 2000000
```

The script writes to every servo, then re-opens at the new baudrate to verify.
You only do this once per set of servos. `config/gello_crx.yaml` already
defaults to `baudrate: 2000000`.

## Step 1 -- Calibrate the leader

Robot disconnected. This only talks to the Dynamixel bus.

```bash
ros2 run gello_crx gello_calibrate --port /dev/ttyUSB0
```

Three stages:

1. **Signs** -- move each joint in the direction described, at least 20 degrees.
2. **Zero** -- hold the leader in the CRX zero pose (upper arm vertical, forearm
   horizontal and forward, wrist straight). Use your printed cradle.
3. **Trigger** -- release fully, then squeeze fully.

It prints a YAML block. Paste it over the `gello_leader` section of
`config/gello_crx.yaml`, then rebuild:

```bash
cd ~/ros2_ws && colcon build --packages-select gello_crx && source install/setup.bash
```

The zero stage warns if any joint sits within 60 degrees of the 0/4095 encoder
seam. Re-clock those horns before continuing -- a seam inside your working range
produces a full-scale jump mid-motion.

## Step 2 -- Verify in mock

```bash
$(ros2 pkg prefix gello_crx)/share/gello_crx/scripts/run_teleop.sh
```

The script runs preflight checks, starts the driver, waits for `/joint_states`
and the controller, starts the three GELLO nodes, prints your current leader
pose in degrees, then waits for you to press Enter before engaging.

Once engaged, watch the state line go `IDLE -> SYNCING -> ENGAGED` and check
the mapping in RViz. **Move one joint at a time.** If a joint moves the wrong
way, flip its entry in `joint_signs` and rebuild. Every sign and offset error
is visible here, and mock costs nothing to get wrong.

Ctrl-C to stop.

## Step 3 -- Real hardware

Lower `velocity_scale` to `0.10` in the config first, then rebuild.

```bash
./run_teleop.sh --real --ip 192.168.1.100
```

The script pings the controller, prints a safety checklist and requires you to
type `CONFIRM`. Before you do:

- DCS zones set around the demo workspace
- Enabling switch wired into the safety circuit and within reach
- Leader held near the robot pose

Raise `velocity_scale` in steps of 0.05 once it feels predictable.

## Script options

| Flag | Effect |
|---|---|
| `--mock` | Mock hardware (default) |
| `--real` | Real controller, adds the confirmation gate |
| `--ip ADDR` | Controller IP, default `192.168.1.100` |
| `--model NAME` | Robot model, default `crx10ia_l` |
| `--port DEV` | Leader serial port, default `/dev/ttyUSB0` |
| `--no-rviz` | Skip RViz |
| `--no-gripper` | Skip the Robotiq bridge |

Logs land in `/tmp/fanuc_driver.log` and `/tmp/gello_crx.log`.

## Running it by hand

Four terminals, if you prefer to see each piece:

```bash
# 1 - driver
ros2 launch fanuc_forward_command fanuc_forward_command.launch.py \
    robot_model:=crx10ia_l use_mock:=true

# 2 - gello nodes
ros2 launch gello_crx gello_teleop.launch.py use_gripper:=false

# 3 - enable heartbeat
ros2 topic pub -r 5 /gello_teleop/enable std_msgs/msg/Bool "{data: true}"

# 4 - watch
ros2 topic echo /gello_teleop/state
```

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/gello_leader/leader_states` | `sensor_msgs/JointState` | out |
| `/gello_leader/trigger` | `std_msgs/Float64` (0..1) | out |
| `/gello_teleop/enable` | `std_msgs/Bool` (heartbeat) | in |
| `/gello_teleop/state` | `std_msgs/String` | out |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | out |
| `/joint_states` | `sensor_msgs/JointState` | in |

## Troubleshooting

**`no /joint_states`** -- check `/tmp/fanuc_driver.log`. On real hardware this
usually means the controller is not in AUTO, the pendant is not disabled, or the
External Control Package licence is not loaded.

**`force sensor spawner failed`** -- expected if the option is not fitted.
Harmless.

**Stuck in `SYNCING`** -- the leader and robot disagree by more than the ramp
can close at `sync_velocity_scale`. Move the leader toward the robot pose, or
raise the scale.

**Flicking to `IDLE`** -- the leader bus is dropping packets or the heartbeat is
stalling. Check the leader node error count in `/tmp/gello_crx.log`.

**Follower feels mushy** -- raise `filter_cutoff_hz` above 5.0, or raise
`velocity_scale`. Jittery instead means the opposite.

## Tuning

`velocity_scale` is the knob that matters. At 0.25 the commanded rate tops out
at 30 deg/s on J1/J2 and 45 deg/s on J3-J6.

`filter_cutoff_hz` at 5 Hz removes encoder quantisation noise -- 0.088 deg per
count is about 2.2 mm of TCP quantisation at full extension -- without adding
lag you can feel.

## Gripper units

`ros2_robotiq_gripper` commands `finger_joint` in radians, roughly 0.0 open to
0.7 closed on a 2F-140. Some forks command stroke in metres instead. Send one
manual goal and confirm before wiring the trigger.

---

## Git workflow for this project

This section explains how Git and GitHub manage this codebase -- how changes
flow from your editor to the robot.

### The big picture

```
 [Your editor]                   [GitHub]                    [Linux laptop]
      |                             |                             |
      |  git add + git commit       |                             |
      |  (save snapshot locally)    |                             |
      |                             |                             |
      |  git push ---- upload ----->|                             |
      |                             |   git pull                  |
      |                             |<----- download latest ------|
```

GitHub is the **central hub**. You push from whatever machine you are coding on,
and pull from whatever machine needs the latest code. Your Windows desktop and
Linux laptop both talk to the same GitHub repo -- they never need to talk
directly to each other.

### Key concepts

| Concept | What it is |
|---|---|
| **Repository (repo)** | The project folder plus its full history. Every clone is a complete copy. |
| **Commit** | A snapshot of all tracked files at one point in time. Has a unique hash (like `c710d01`), a message, and a pointer to its parent. |
| **Branch** | A named pointer to one commit. `main` is the default. Branches let you work on changes without affecting the stable version. |
| **Remote** | A copy of the repo hosted elsewhere. `origin` = GitHub. Your local repo and `origin` are independent until you push/pull. |
| **Working directory** | The actual files on disk. Can differ from the last commit (you have edited but not committed yet). |
| **Staging area** | A holding zone between your edits and the next commit. `git add` moves changes here. |

### Daily workflow (solo developer)

For this project you are the only developer, so you can commit directly to
`main`. The workflow is:

```bash
# 1. Make your edits (change gello_crx.yaml, fix a node, etc.)

# 2. See what changed
git status                    # which files changed?
git diff                      # what exactly changed in each file?

# 3. Stage the changes you want to commit
git add config/gello_crx.yaml gello_crx/teleop_node.py
#   or stage everything:
git add -A

# 4. Commit with a meaningful message
git commit -m "tune: raise velocity_scale to 0.30 for demo"

# 5. Push to GitHub
git push

# 6. On the Linux laptop, pull the changes
cd ~/ros2_ws/src/gello_crx
git pull
cd ~/ros2_ws && colcon build --packages-select gello_crx && source install/setup.bash
```

### When to use branches

Branches are useful when you want to try something without risking the stable
version. Example: testing a new filter algorithm.

```bash
# Create and switch to a new branch
git checkout -b experiment/butterworth-filter

# Make changes, commit as usual
git add -A
git commit -m "experiment: try second-order Butterworth filter"

# Push the branch to GitHub
git push -u origin experiment/butterworth-filter

# If the experiment works, merge it into main:
git checkout main
git merge experiment/butterworth-filter
git push

# If it does not work, just switch back:
git checkout main
# The experiment branch stays if you want to revisit it later
```

### Useful commands reference

```bash
git status              # What has changed since the last commit?
git log --oneline -10   # Last 10 commits, one line each
git diff                # Show unstaged changes
git diff --cached       # Show staged changes (after git add)
git stash               # Temporarily shelve uncommitted changes
git stash pop           # Bring them back
git branch              # List local branches
git branch -a           # List all branches (including remote)
git remote -v           # Show where push/pull go (should be GitHub)
```

### What lives in Git vs what does not

| In the repo (tracked) | NOT in the repo (in .gitignore) |
|---|---|
| Source code (`.py`) | Build artifacts (`build/`, `install/`) |
| Config files (`.yaml`) | Python bytecode (`__pycache__/`) |
| Launch files | IDE settings (`.vscode/`, `.idea/`) |
| Shell scripts | Log files |
| STL/CAD models | Compiled libraries (`.so`, `.o`) |
| `package.xml`, `setup.py` | |

### ROS 2 workspace layout

```
~/ros2_ws/                      <-- workspace root
|-- build/                      <-- colcon output (auto-generated, gitignored)
|-- install/                    <-- installed packages (auto-generated, gitignored)
|-- log/                        <-- build logs (auto-generated, gitignored)
+-- src/                        <-- your packages (each can be its own git repo)
    |-- gello_crx/              <-- THIS repo
    |   |-- gello_crx/          <-- Python package
    |   |-- config/
    |   |-- launch/
    |   |-- package.xml
    |   +-- setup.py
    +-- fanuc_forward_command/  <-- the FANUC driver (separate repo)
```

Each package in `src/` can be its own git repo. `colcon build` finds them all
by scanning for `package.xml` files. The `build/`, `install/`, and `log/`
directories are generated by colcon and should never be committed.

## Not included

Demo recording. The insertion point is a fourth node subscribing to
`/gello_leader/leader_states`, `/joint_states`, the trigger and your RealSense
topics, writing LeRobot-format episodes.
