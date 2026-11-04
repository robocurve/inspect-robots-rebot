# Two-arm reBot adapter

`rebot_arms` is the Cartesian counterpart to `yam_arms`. It accepts 14 absolute
values, left arm then right arm: `x, y, z, yaw, pitch, roll, gripper`.
Each arm uses its own base frame. XYZ is in metres at the URDF gripper end.
Rotation is `Rz(yaw) Ry(-pitch) Rx(roll)` relative to the reset orientation.
Pitch and roll are fixed at zero. The gripper uses explicit closed/open
calibration in motor radians; its policy range is 0 to 1.

The package keeps the original `b601` single-arm entry point and operator CLI.
Do not install this distribution beside `inspect-robots-b601`: both contain
`inspect_robots_b601`. Use a separate environment for migration.

## Installation

```sh
python -m pip install '.[cartesian,cameras]'
```

The streaming session requires Motorbridge's `get_state_snapshot()` API **and
its matching native ABI**. Version 0.5.6 alone does not establish that capability.
The API is present in the public Motorbridge source at
[`d5700bcd`](https://github.com/motorbridge/motorbridge/tree/d5700bcd5b7f9355dd2012488d463c3bac982259).
Build/install both the Python SDK and native library from that source using its
build instructions, or use a tested wheel with the same API. The legacy
`motorbridge` extra retains the old pin for single-arm compatibility; it is not
sufficient evidence that streaming is supported. `open()` checks both the Python
method and the ABI before any motor is enabled. A missing snapshot is permitted
only before activation; fresh disabled-state replies are required at activation.

Example construction (site-specific paths are examples):

```python
from inspect_robots_rebot import RebotEmbodiment

robot = RebotEmbodiment(
    left_channel="can1",
    right_channel="can0",
    left_profile="/site/left.json",
    right_profile="/site/right.json",
    left_state_dir="/site/state/left",
    right_state_dir="/site/state/right",
    camera_url="http://camera-host:1984/api/stream.mjpeg?src=top",
    motor_temp_limit=80,
    tool_floor_m=0.12,  # Set from the installation base-frame/tabletop calibration.
    allow_motion=False,
)
```

Construction opens no CAN interface, camera, or worker thread. Keep the existing
stop directories during migration. Do not create fresh directories to bypass a
latched stop. The operator CLI provides explicit recovery after fault inspection.
Profiles retain the CAN ownership locks, motor IDs, bounds, torque caps, and gains.
If USB serial numbers are duplicated, identify the physical left/right ports
before configuration; interface names alone do not prove arm identity.

## Control and limits

- The policy loop is paced at 10 Hz. Motor holding uses the native background loop.
- `step()` does not wait for settling or a new full round of position reads.
  Observations use cached replies with driver-provided age; the oldest state time
  is reported. Feedback older than 0.1 seconds is rejected.
- Bounded least-squares IK can return an approximate solution. It reports position
  and rotation residuals. A finite result inside bounds does not prove a target
  was reached. Read the measured `eef_state`.
- The command is limited to 0.08 rad ahead of arm feedback and 0.44 rad ahead of
  gripper feedback. The native command-lead allowances are 0.09 and 0.45 rad.
  These are not motor fault thresholds. The native loss-of-control envelopes
  remain 0.6 rad for arm joints and 5 rad for the gripper. Feedback is checked
  against those envelopes before a command is sent.
- No additional velocity, acceleration, or jerk planner runs on policy targets.
  The tracking allowance still affects attainable speed. Native homing retains
  its separate slow path behavior. This does not promise YAM's physical speed.
- `tool_floor_m` sets both arms' lower Cartesian target bound and the sampled
  joint-path floor. Its default is 0.12 m. Values are in each arm's base frame.
  A station with the same mounting reference as rig 1 can set `tool_floor_m=-0.02`
  to match rig 1's configured target lower bound. Confirm the tabletop height
  before transferring a value between installations. This sampled tool check is
  not a complete collision check. There is no inter-arm or whole-link model.
- The default temperature stop is 80 C. Torque caps, joint bounds, motor status,
  fresh feedback and the motor command watchdog remain active.
- Limits apply inside the adapter, including direct programmatic use. The action
  space does not add a command-history delta limiter. Gripper increments use
  measured motor feedback, so a modified command cannot advance a separate
  unexecuted gripper reference. Custom external approvers remain the caller's
  responsibility.

`StepResult.info` includes `arms` (IK residuals and `tracking_limited`),
`commanded_motor_rad`, and `step_duration_s`. A tracking limit is not reported as
successful arrival. Commands are submitted to independent CAN buses; submission
is not hardware-atomic. On a submission error, both sessions are held and closed.

## Lifecycle

Reset validates the camera, opens both buses, then activates both arms. A failure
at any stage closes all acquired resources. Each native control loop checks the
shared pair-stop event, so a fault on one arm inhibits the other. Close attempts
both native home sequences independently, then disables and releases resources.
Disable confirmation requires a **new** reply from every motor with disabled
mode bits. Missing replies and cleanup errors are retained in `shutdown_reports`
and make close fail. A log failure cannot prevent resource cleanup.

The native homing path is retained, not mechanically requalified by this change.
A blocked home can time out; disable can then remove gravity support. The physical
installation must support this shutdown behavior. Automated tests cannot verify
mounts, wiring, collision clearance, calibration, or safe unattended operation.

## Assessment of the original adapter

Keep: source-traced geometry and gravity model, exclusive CAN ownership, durable
stop records, torque caps, explicit recovery, and injected hardware test seams.

Replace for two-arm policy use: single-arm raw-joint actions, blocking move/settle
calls, and station scripts that replace classes at runtime. The new public session
adds position streaming and feedback snapshots without runtime class replacement.
The original single-arm interface stays available for commissioning.

The YAM adapter is an interface reference, not a motor controller to copy.
Its Cartesian path includes an IK joint increment limit (default 0.2 rad), command
resynchronization, and oscillation handling. It is incorrect to describe it as
having no limits. These reBot tracking limits are different and are documented
above rather than presented as identical YAM behavior.

## Qualification

Hardware-free tests cover lifecycle rollback, both-arm cleanup, finite/bounded IK,
joint-boundary solutions, calibrated grippers, stale/invalid feedback, temperature
and torque faults, command-lead margin, native envelope faults, disable acknowledgement, and camera failures.
Before an overnight queue, test the installed package and driver together, inspect
both-arm motion and shutdown, and complete a short live RoboComms trial. Record the
package commit, driver build, station profiles, and trial result outside this public
repository. Do not publish credentials, private queue data, or site addresses here.


## Command reference comparison

The default `command_mode="measured"` retains the 0.08 rad arm command
lead relative to measured position. The optional `command_mode="previous"`
limits each arm command change to `command_step_rad=0.2` rad from the prior
issued command. If any of the six arm joints differs from that reference by
more than `command_resync_rad=0.35` rad, all six references reset to measured
position. The first command also starts from measured position. The gripper
retains its separate measured-position allowance.

These parameters follow the YAM command-step and reference-reset method.
They do not reproduce YAM's IK solver or oscillation hold, and do not prove
equal physical speed. Their sum must stay below the native 0.6 rad fault envelope.
Joint position bounds and feedback checks remain active. Rejected paths do
not advance the issued-command reference. Step records include requested and
issued motor targets, command limiting, reference resets, and elapsed time.

Both command modes passed small near-home checks on one installation.
This does not qualify arbitrary trajectories or prove speed parity. Resolve
boundary overshoot before full-workspace speed qualification. Compare equivalent interior-workspace trajectories and measure
actual movement time separately from model inference time.

## Wrist yaw endpoint clearance

`wrist_yaw_command_margin_rad` narrows only joint 5's IK command range at
both ends. It defaults to zero. It does not change the motor profile, measured
position tolerance, torque limits, feedback checks, or other joint ranges.
The solver clips its seed to the reduced range, including when the measured
position starts outside that command range. Invalid margins fail at construction.

The manufacturer lists joint 5 travel as +/-90 degrees:
https://wiki.seeedstudio.com/rebot_arm_b601_rs_pinocchio_meshcat/
This matches the shipped profile. It does not specify allowable dynamic
overshoot beyond that range. A station can use endpoint clearance to allow
tracking error without commanding the exact range endpoint. Clearance requires
hardware validation and is not a replacement for the measured-position stop.

For an installation with a reduced joint 2 operating range,
`joint2_upper_tolerance_rad` can be set from 0.010 to 0.012 rad. The default is
0.010 rad. A larger value is rejected unless the upper command bound plus the
tolerance remains inside the stock profile's 170-degree upper bound. This
option changes only the upper measured-position allowance for joint 2.
It does not change commands, lower bounds, other joints, or other fault checks.

## Controller temperature and stalled commands

`rs00_mos_temp_limit` sets a separate MOS/controller temperature cutoff for
RS-00 motors, up to 100 C. By default it follows `motor_temp_limit`. The general
limit remains capped at 80 C and still applies to RS-06 MOS readings and all
rotor readings. A deployment must establish which sensor the manufacturer's
operating advice covers; support for a configuration value is not a hardware
rating or a guarantee that a stalled pose can be held indefinitely.

The streaming loop now holds an arm joint at its measured position when effort
is at least 90% of its configured cap, command error exceeds 0.08 rad, and the
joint fails to move by more than 0.005 rad for 0.5 seconds. This also runs while
the policy waits for model inference. State `blocked` reports the direction;
a command more than 0.01 rad in the reverse direction releases the block.
The gripper and native parking path retain their existing behavior. Normal
moving joints retain the configured command step. Fault checks run before this
stall response, so it cannot hide a position, temperature, or tracking fault.
