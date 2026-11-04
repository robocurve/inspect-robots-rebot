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
  gripper feedback. The corresponding tracking stops remain 0.09 and 0.45 rad.
  This 0.01 rad margin avoids commanding exactly at the fault boundary.
- No additional velocity, acceleration, or jerk planner runs on policy targets.
  The tracking allowance still affects attainable speed. Native homing retains
  its separate slow path behavior. This does not promise YAM's physical speed.
- The sampled commanded joint segment must keep the tool above 0.12 m. This is
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
and torque faults, tracking margin, disable acknowledgement, and camera failures.
Before an overnight queue, test the installed package and driver together, inspect
both-arm motion and shutdown, and complete a short live RoboComms trial. Record the
package commit, driver build, station profiles, and trial result outside this public
repository. Do not publish credentials, private queue data, or site addresses here.
