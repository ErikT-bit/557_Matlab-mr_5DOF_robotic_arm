# 557 Robot Arm MATLAB

This repository contains the cleaned MATLAB workflow for the ME557 whiteboard-writing robot.

The project uses a MATLAB + Modern Robotics workflow to:
- load the robot model from a URDF,
- capture a whiteboard plane from 4 measured points,
- convert a sketched drawing into board-space waypoints,
- solve the drawing path in joint space with iterative inverse kinematics,
- and execute the resulting motion on the physical robot.

This repository is organized so that the main files needed to run the whiteboard-writing workflow are separated from unused test scripts and older hardware-control files.

## Folder layout

### `matlab/main/`
Main entry-point scripts to run.

### `matlab/hardware/`
RB150 communication, calibrated joint conversion, servo mapping, limit handling, and robot-specific motor-coupling behavior.

### `matlab/planning/`
Robot model loading, board-plane fitting, task-space conversion, and IK trajectory generation.

### `matlab/ui/`
Viewer and sketch capture tools.

### `matlab/utils/`
Optional startup or small support utilities.

### `robot_description/urdf/`
Robot URDF files.

### `robot_description/meshes/`
STL meshes referenced by the URDF.

### `robot_description/config/`
YAML, SRDF, RViz, and related robot configuration files.

### `mr/`
Only the Modern Robotics MATLAB functions required by this workflow.

### `calibration/`
User-specific calibration files.

### `arduino/`
Arduino sketches used for low-level RB150 / Dynamixel communication and any robot-specific motor coordination logic.

## Before running

Check these user-specific items first.

### 1. COM port

Edit this line in `matlab/main/main_writeboard_demo.m`:

```matlab
hw = robot_hw_rb150_calibrated_6motor("COM13", 1000000);
```

Change `"COM13"` to the COM port on your PC.

### 2. URDF link and joint names

`main_writeboard_demo.m` assumes these names exist in the URDF:

**Base link:**
- `FixedBase`

**Tip link:**
- `PenTipLink`

**Joint names:**
- `RotatingBaseJoint`
- `ShoulderJoint`
- `ElbowJoint`
- `WristJoint`
- `PenJoint`

If your URDF uses different names, edit the variables in `main_writeboard_demo.m`.

### 3. Calibration and hardware mapping

The hardware mapping is robot-specific. Review:
- `matlab/hardware/servo_calibration.m`
- `calibration/servo_cal_user.m`
- `calibration/servo_cal_user.mat`

If your horn zero positions, motor directions, offsets, limits, motor IDs, or the joint-2 / joint-3 tandem relationship are different, update these files and the related hardware-control files before running.

### 4. Coupled motor behavior for joints 2 and 3

This robot uses a mechanically coupled / mirrored arrangement for motors 2 and 3.

That means the low-level hardware control must preserve the intended relationship between those two motors.

The MATLAB kinematics treat the arm as a 5-DOF serial chain, but the hardware layer is responsible for converting those joint commands into the correct motor commands for the physical robot.

Review these files carefully before running on a different robot design:
- `matlab/hardware/robot_hw_rb150_calibrated_6motor.m`
- `matlab/hardware/robot_hw_rb150_raw6motor.m`
- `matlab/hardware/moveit_rad_to_servo.m`
- `matlab/hardware/servo_to_moveit_rad.m`
- `matlab/hardware/theta_to_ax_goalpos.m`
- `matlab/hardware/servo_calibration.m`
- `arduino/arduino_to_matlab_FLASH_THIS_TO_RB150.ino`

If your robot does not use the same tandem / mirrored motor arrangement for joints 2 and 3, these files may need to be modified before any motion is attempted.

Examples of things that may differ between robot builds:
- whether motor 2 and motor 3 should move equal-and-opposite,
- whether they should move equal-in-the-same-direction,
- sign conventions,
- motor ID ordering,
- zero offsets,
- servo limits,
- gear or linkage ratios.

Do not assume this repo is plug-and-play for a different physical arm unless the joint-2 / joint-3 coupling matches your hardware.

### 5. Home and waypoint settings

These values are robot-specific and should be checked before first motion:
- `theta_home`
- `theta_waypoint1`

### 6. Board / pen settings

These may need adjustment for your setup:
- `planePressOffset_m`
- `boardBackoff_m`
- `penUpHeight_m`
- `desiredTextHeight_m`
- `boardSize_m`

### 7. Folder structure

This repo uses relative paths so it can be moved between computers more easily.

Keep the folder structure intact unless you also update the path logic inside the MATLAB scripts.

## Main workflow

Run from MATLAB:

```matlab
main_writeboard_demo
```

## Step-by-step function map

### Step 1 - Go HOME

**Purpose:** torque on, read current joints, and move to the home posture if needed.

**Calls:**
- `robot_hw_rb150_calibrated_6motor`
- `visualize_robot`
- local helper `move_slow_quintic_limited`

### Step 2 - Plane capture

**Purpose:** manually touch 4 board points and convert joint states into 3D board points.

**Calls:**
- `hw.readJoints`
- `FKinSpace`
- `plane_from_4pts`
- local helper `board_frame_from_points`
- `viewer.setBoard`

### Step 3 - Move to waypoint1

**Purpose:** move to a known intermediate posture before sketch capture.

**Calls:**
- local helper `move_slow_quintic_limited`

### Step 4 - Sketch capture

**Purpose:** capture drawing strokes from the sketchpad window.

**Calls:**
- `sketchpad_capture`

### Step 5 - Scale and center

**Purpose:** scale the sketch to the desired text height.

**Calls:**
- `strokes_scale_center`

### Step 6 - Strokes to task space

**Purpose:** map 2D sketch strokes to 3D poses on the fitted board plane.

**Calls:**
- `strokes_to_taskspace`
- `viewer.setPath`

### Step 7 - IK solve

**Purpose:** solve the drawing path waypoint-by-waypoint in joint space.

**Calls:**
- `run_ik_trajectory`

Inside that pipeline, the custom solver uses a position-only IK workflow supported by Modern Robotics helper functions.

Modern Robotics functions used in this pipeline include:
- `FKinSpace`
- `JacobianSpace`
- `MatrixLog6`
- `TransInv`
- `Adjoint`
- `se3ToVec`
- `MatrixExp6`
- `VecTose3`

### Step 8 - Execute drawing

**Purpose:** move to the first waypoint, then execute the joint trajectory.

**Calls:**
- local helper `move_slow_quintic_limited`
- local helper `execute_trajectory_safe`

**Important note:**  
Joint-space commands are converted into physical motor commands through the hardware layer, which may include robot-specific coupling logic for motors 2 and 3.

### Step 9 - Return HOME

**Purpose:** move the arm back to home after drawing.

## Main function dependency chain

The main script follows this general dependency chain:

```text
main_writeboard_demo
 ├─ robot_model_from_urdf
 ├─ robot_hw_rb150_calibrated_6motor
 ├─ visualize_robot
 ├─ plane_from_4pts
 ├─ sketchpad_capture
 ├─ strokes_scale_center
 ├─ strokes_to_taskspace
 ├─ run_ik_trajectory
 │   └─ ik_position_only_space
 │       └─ Modern Robotics helper functions
 └─ hardware execution helpers
```

## Files most likely to require user edits

If another person is trying to adapt this repo to a different robot, these are the first files they should review:
- `matlab/main/main_writeboard_demo.m`
- `matlab/hardware/servo_calibration.m`
- `calibration/servo_cal_user.m`
- `calibration/servo_cal_user.mat`
- `matlab/hardware/robot_hw_rb150_calibrated_6motor.m`
- `matlab/hardware/robot_hw_rb150_raw6motor.m`
- `matlab/hardware/moveit_rad_to_servo.m`
- `matlab/hardware/servo_to_moveit_rad.m`
- `matlab/hardware/theta_to_ax_goalpos.m`
- `arduino/arduino_to_matlab_FLASH_THIS_TO_RB150.ino`
- `robot_description/urdf/simple_robot_v2.urdf`

## Notes

- This cleaned repo is organized around the whiteboard-writing workflow only.
- Older single-motor and unused hardware test files were intentionally removed from this repo.
- The Modern Robotics functions in `mr/` are only the subset needed by the current workflow.
- The MATLAB layer assumes a 5-DOF kinematic model, while the physical hardware layer may still require robot-specific actuator coupling and calibration.
