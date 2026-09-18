#!/usr/bin/env bash
# Bring up GELLO teleoperation for the FANUC CRX-10iA/L.
#
#   ./run_teleop.sh                      # mock hardware, RViz on
#   ./run_teleop.sh --real --ip 10.0.0.5 # real controller
#   ./run_teleop.sh --no-rviz --no-gripper
#
# Enable is a heartbeat. This script publishes it at 5 Hz while running and
# stops on exit, so Ctrl-C or a closed terminal halts the robot within 0.5 s.

set -uo pipefail

USE_MOCK=true
ROBOT_IP="192.168.1.100"
ROBOT_MODEL="crx10ia_l"
LEADER_PORT="/dev/ttyUSB0"
LAUNCH_RVIZ=true
USE_GRIPPER=true
STARTUP_TIMEOUT=120

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real)        USE_MOCK=false; shift ;;
    --mock)        USE_MOCK=true; shift ;;
    --ip)          ROBOT_IP="$2"; shift 2 ;;
    --model)       ROBOT_MODEL="$2"; shift 2 ;;
    --port)        LEADER_PORT="$2"; shift 2 ;;
    --no-rviz)     LAUNCH_RVIZ=false; shift ;;
    --no-gripper)  USE_GRIPPER=false; shift ;;
    -h|--help)     sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

log()  { printf '\033[1;36m[teleop]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[fail]\033[0m %s\n' "$*" >&2; exit 1; }

DRIVER_PID=""
GELLO_PID=""
HEARTBEAT_PID=""

cleanup() {
  echo
  log "shutting down"
  # Kill the heartbeat first: teleop drops to IDLE 0.5 s later and the
  # controller holds its last setpoint. Nothing else should move.
  [[ -n "$HEARTBEAT_PID" ]] && kill "$HEARTBEAT_PID" 2>/dev/null
  sleep 0.8
  timeout 3 ros2 topic pub -1 /gello_teleop/enable std_msgs/msg/Bool \
    "{data: false}" >/dev/null 2>&1
  [[ -n "$GELLO_PID"  ]] && kill -INT "$GELLO_PID"  2>/dev/null
  [[ -n "$DRIVER_PID" ]] && kill -INT "$DRIVER_PID" 2>/dev/null
  sleep 2
  [[ -n "$GELLO_PID"  ]] && kill -9 "$GELLO_PID"  2>/dev/null
  [[ -n "$DRIVER_PID" ]] && kill -9 "$DRIVER_PID" 2>/dev/null
  log "done"
}
trap cleanup EXIT INT TERM

wait_for_topic() {   # name, timeout
  local topic="$1" limit="$2" waited=0
  while ! ros2 topic list 2>/dev/null | grep -qx "$topic"; do
    sleep 1; waited=$((waited + 1))
    [[ $waited -ge $limit ]] && return 1
  done
  return 0
}

wait_for_message() { # name, timeout
  timeout "$2" ros2 topic echo --once "$1" >/dev/null 2>&1
}

# ---------------------------------------------------------------- preflight

log "preflight"

command -v ros2 >/dev/null || die "ros2 not on PATH - source your ROS 2 setup first"
ros2 pkg prefix gello_crx  >/dev/null 2>&1 || die "gello_crx not found - colcon build and source install/setup.bash"
ros2 pkg prefix fanuc_forward_command >/dev/null 2>&1 || die "fanuc_forward_command not found"

python3 -c "import dynamixel_sdk" 2>/dev/null || die "dynamixel_sdk not importable"

[[ -e "$LEADER_PORT" ]] || die "leader not found at $LEADER_PORT (check the U2D2 and try dmesg | tail)"
[[ -r "$LEADER_PORT" && -w "$LEADER_PORT" ]] || \
  die "no read/write on $LEADER_PORT - run: sudo usermod -aG dialout \$USER, then log out and back in"

CONFIG="$(ros2 pkg prefix gello_crx)/share/gello_crx/config/gello_crx.yaml"
if grep -q 'joint_offsets: \[0.0, 0.0, 0.0, 0.0, 0.0, 0.0\]' "$CONFIG" 2>/dev/null; then
  warn "joint_offsets are still placeholder zeros in $CONFIG"
  warn "run: ros2 run gello_crx gello_calibrate --port $LEADER_PORT"
  read -rp "continue anyway? [y/N] " reply
  [[ "$reply" == "y" || "$reply" == "Y" ]] || exit 1
fi

if [[ "$USE_MOCK" == "false" ]]; then
  ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1 || warn "no ping response from $ROBOT_IP"
  cat <<'BANNER'

  ============================================================
   REAL HARDWARE. Confirm before continuing:
     - DCS zones set around the demo workspace
     - enabling switch in the safety circuit and within reach
     - nobody inside the cell
     - leader held near the robot's current pose
     - velocity_scale lowered for a first session
  ============================================================

BANNER
  read -rp "type CONFIRM to proceed: " reply
  [[ "$reply" == "CONFIRM" ]] || exit 1
fi

# ------------------------------------------------------------------- driver

log "starting FANUC driver (model=$ROBOT_MODEL mock=$USE_MOCK)"
ros2 launch fanuc_forward_command fanuc_forward_command.launch.py \
  robot_model:="$ROBOT_MODEL" \
  use_mock:="$USE_MOCK" \
  robot_ip:="$ROBOT_IP" \
  launch_rviz:="$LAUNCH_RVIZ" \
  >/tmp/fanuc_driver.log 2>&1 &
DRIVER_PID=$!

log "waiting for /joint_states (up to ${STARTUP_TIMEOUT}s)"
wait_for_topic /joint_states "$STARTUP_TIMEOUT" || die "no /joint_states - see /tmp/fanuc_driver.log"
wait_for_message /joint_states 20 || die "/joint_states exists but is silent - see /tmp/fanuc_driver.log"

wait_for_topic /forward_position_controller/commands 30 || \
  die "forward_position_controller never came up - see /tmp/fanuc_driver.log"
log "driver up"

# The force-sensor spawners fail on robots without the option fitted. Harmless.
grep -q "force_sensor.*failed\|force_torque.*failed" /tmp/fanuc_driver.log 2>/dev/null && \
  warn "force sensor spawner failed (expected if the option is not fitted)"

# -------------------------------------------------------------------- gello

log "starting gello_crx nodes"
ros2 launch gello_crx gello_teleop.launch.py \
  use_gripper:="$USE_GRIPPER" \
  >/tmp/gello_crx.log 2>&1 &
GELLO_PID=$!

wait_for_topic /gello_leader/leader_states 30 || die "leader node did not start - see /tmp/gello_crx.log"
wait_for_message /gello_leader/leader_states 10 || die "leader bus is silent - see /tmp/gello_crx.log"
wait_for_topic /gello_teleop/state 30 || die "teleop node did not start - see /tmp/gello_crx.log"
log "leader streaming"

echo
log "current leader pose (deg):"
timeout 5 ros2 topic echo --once /gello_leader/leader_states 2>/dev/null \
  | python3 -c "
import sys, math
for line in sys.stdin:
    if line.startswith('position:'):
        break
vals = []
for line in sys.stdin:
    s = line.strip()
    if not s.startswith('-'):
        break
    vals.append(float(s.lstrip('- ')))
print('   ' + '  '.join(f'J{i+1}={math.degrees(v):+7.1f}' for i, v in enumerate(vals[:6])))
" 2>/dev/null || echo "   (could not parse)"

echo
log "ready to engage"
warn "the robot will move to match the leader once enabled"
read -rp "press Enter to engage, Ctrl-C to abort "

# ---------------------------------------------------------------- heartbeat

log "publishing enable heartbeat at 5 Hz"
ros2 topic pub -r 5 /gello_teleop/enable std_msgs/msg/Bool "{data: true}" \
  >/dev/null 2>&1 &
HEARTBEAT_PID=$!

echo
log "TELEOP ACTIVE - Ctrl-C to stop"
log "state:  ros2 topic echo /gello_teleop/state"
log "logs:   /tmp/fanuc_driver.log  /tmp/gello_crx.log"
echo

# Surface state transitions so a drop to IDLE is visible rather than silent.
ros2 topic echo /gello_teleop/state --field data 2>/dev/null \
  | awk '$0 != last { print strftime("[%H:%M:%S]"), $0; last = $0 }' &
STATE_PID=$!

wait "$HEARTBEAT_PID"
kill "$STATE_PID" 2>/dev/null
