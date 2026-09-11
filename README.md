# gello_crx

GELLO leader-arm teleoperation for the FANUC CRX-10iA/L. Plain `rclpy`, three
nodes, no ZMQ and no GELLO runtime dependency.

```
gello_leader ──/gello_leader/leader_states──► gello_teleop ──► /forward_position_controller/commands
     │                                              ▲
     └────────/gello_leader/trigger──► gello_gripper│
                                                    │
                              /joint_states ────────┘
```

---

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

---

## Step 0 — One-time setup

```bash
# Artifactory, not PyPI (Lilly policy)
pip install --index-url https://elilillyco.jfrog.io/artifactory/api/pypi/pypi/simple \
    dynamixel-sdk

sudo usermod -aG dialout $USER    # log out and back in afterwards

cd ~/ros2_ws/src && cp -r /path/to/gello_crx .
cd ~/ros2_ws && colcon build --packages-select gello_crx
source install/setup.bash
```

Confirm the leader is on the bus:

```bash
ls -l /dev/ttyUSB*
```

---

## Step 1 — Calibrate the leader

Robot disconnected. This only talks to the Dynamixel bus.

```bash
ros2 run gello_crx gello_calibrate --port /dev/ttyUSB0
```

Three stages:

1. **Signs** — move each joint in the direction described, at least 20 degrees.
2. **Zero** — hold the leader in the CRX zero pose (upper arm vertical, forearm
   horizontal and forward, wrist straight). Use your printed cradle.
3. **Trigger** — release fully, then squeeze fully.

It prints a YAML block. Paste it over the `gello_leader` section of
`config/gello_crx.yaml`, then rebuild:

```bash
cd ~/ros2_ws && colcon build --packages-select gello_crx && source install/setup.bash
```

The zero stage warns if any joint sits within 60 degrees of the 0/4095 encoder
seam. Re-clock those horns before continuing — a seam inside your working range
produces a full-scale jump mid-motion.

---

## Step 2 — Verify in mock

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

---

## Step 3 — Real hardware

Lower `velocity_scale` to `0.10` in the config first, then rebuild.

```bash
./run_teleop.sh --real --ip 192.168.1.100
```

The script pings the controller, prints a safety checklist and requires you to
type `CONFIRM`. Before you do:

- DCS zones set around the demo workspace
- Enabling switch wired into the safety circuit and within reach. The driver
  requires AUTO with the pendant disabled, so there is no deadman by default —
  wire the switch so it can interrupt the heartbeat, not just your attention.
- Leader held near the robot's current pose. The sync ramp handles a mismatch,
  but at `sync_velocity_scale: 0.05` a 90 degree error means about 15 seconds
  of the robot moving on its own before it starts following you.

Raise `velocity_scale` in steps of 0.05 once it feels predictable.

---

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

---

## Running it by hand

Four terminals, if you prefer to see each piece:

```bash
# 1 - driver.  use_mock defaults to FALSE, so always pass it explicitly.
ros2 launch fanuc_forward_command fanuc_forward_command.launch.py \
    robot_model:=crx10ia_l use_mock:=true

# 2 - gello nodes
ros2 launch gello_crx gello_teleop.launch.py use_gripper:=false

# 3 - enable heartbeat.  Must repeat; a single -1 publish engages for 0.5 s only.
ros2 topic pub -r 5 /gello_teleop/enable std_msgs/msg/Bool "{data: true}"

# 4 - watch
ros2 topic echo /gello_teleop/state
```

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/gello_leader/leader_states` | `sensor_msgs/JointState` | out |
| `/gello_leader/trigger` | `std_msgs/Float64` (0..1) | out |
| `/gello_teleop/enable` | `std_msgs/Bool` (heartbeat) | in |
| `/gello_teleop/state` | `std_msgs/String` | out |
| `/forward_position_controller/commands` | `std_msgs/Float64MultiArray` | out |
| `/joint_states` | `sensor_msgs/JointState` | in |

---

## Troubleshooting

**`no /joint_states`** — check `/tmp/fanuc_driver.log`. On real hardware this
usually means the controller isn't in AUTO, the pendant isn't disabled, or the
External Control Package licence isn't loaded.

**`force sensor spawner failed`** — expected if the option isn't fitted. The
driver spawns all five controllers unconditionally. Harmless.

**Stuck in `SYNCING`** — the leader and robot disagree by more than the ramp
can close at `sync_velocity_scale`. Move the leader toward the robot's pose, or
raise the scale.

**Flicking to `IDLE`** — the leader bus is dropping packets or the heartbeat is
stalling. Check the leader node's error count in `/tmp/gello_crx.log`.

**Follower feels mushy** — raise `filter_cutoff_hz` above 5.0, or raise
`velocity_scale`. Jittery instead means the opposite.

---

## Tuning

`velocity_scale` is the knob that matters. At 0.25 the commanded rate tops out
at 30 deg/s on J1/J2 and 45 deg/s on J3–J6.

`filter_cutoff_hz` at 5 Hz removes encoder quantisation noise — 0.088 deg per
count is about 2.2 mm of TCP quantisation at full extension — without adding
lag you can feel.

The two interact. A dry run of the pipeline showed a 90 degree leader step
tracked to completion in 4.0 s with the commanded rate pinned at exactly the
30 deg/s ceiling, which is the intended behaviour: the filter smooths, the
limiter caps, and the filter cannot defeat the cap.

---

## Gripper units

`ros2_robotiq_gripper` commands `finger_joint` in radians, roughly 0.0 open to
0.7 closed on a 2F-140. Some forks command stroke in metres instead. Send one
manual goal and confirm before wiring the trigger — the defaults here are a
guess at your stack, not a reading of it.

---

## Not included

Demo recording. The insertion point is a fourth node subscribing to
`/gello_leader/leader_states`, `/joint_states`, the trigger and your RealSense
topics, writing LeRobot-format episodes.
