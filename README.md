# 557 Robot Arm MATLAB (Clean Repo)

This repository contains the cleaned MATLAB workflow for the ME557 whiteboard-writing robot.

## Folder layout

- `matlab/main/`  
  Main entry-point scripts to run.

- `matlab/hardware/`  
  RB150 communication, calibrated joint conversion, and hardware-side helpers.

- `matlab/planning/`  
  Robot model loading, board-plane fitting, task-space conversion, and IK trajectory generation.

- `matlab/ui/`  
  Viewer and sketch capture tools.

- `matlab/utils/`  
  Optional startup or small support utilities.

- `robot_description/urdf/`  
  Robot URDF files.

- `robot_description/meshes/`  
  STL meshes referenced by the URDF.

- `robot_description/config/`  
  YAML, SRDF, RViz, and related robot configuration files.

- `mr/`  
  Only the Modern Robotics MATLAB functions required by this workflow.

- `calibration/`  
  User-specific calibration files.

- `arduino/`  
  Arduino sketches used by the project.

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

- Base link: `FixedBase`
- Tip link: `PenTipLink`

Joint names:
- `RotatingBaseJoint`
- `ShoulderJoint`
- `ElbowJoint`
- `WristJoint`
- `PenJoint`

If your URDF uses different names, edit the variables in `main_writeboard_demo.m`.

### 3. Calibration files
The hardware mapping is robot-specific. Review:

- `matlab/hardware/servo_calibration.m`
- `calibration/servo_cal_user.m`
- `calibration/servo_cal_user.mat`

If your horn zero positions, motor directions, offsets, or limits are different, update these files.

### 4. Home and waypoint settings
These values are robot-specific and should be checked before first motion:

- `theta_home`
- `theta_waypoint1`

### 5. Board / pen settings
These may need adjustment for your setup:

- `planePressOffset_m`
- `boardBackoff_m`
- `penUpHeight_m`
- `desiredTextHeight_m`
- `boardSize_m`

## Main workflow

Run from MATLAB:

```matlab
main_writeboard_demo
```

## Step-by-step function map

### Step 1 - Go HOME
Purpose: torque on, read current joints, and move to the home posture if needed.

Calls:
- `robot_hw_rb150_calibrated_6motor`
- `visualize_robot`
- local helper `move_slow_quintic_limited`

### Step 2 - Plane capture
Purpose: touch 4 board points manually and convert joint states into 3D board points.

Calls:
- `hw.readJoints`
- `FKinSpace`
- `plane_from_4pts`
- local helper `board_frame_from_points`
- `viewer.setBoard`

### Step 3 - Move to waypoint1
Purpose: move to a known intermediate posture before sketch capture.

Calls:
- local helper `move_slow_quintic_limited`

### Step 4 - Sketch capture
Purpose: capture strokes from the sketchpad window.

Calls:
- `sketchpad_capture`

### Step 5 - Scale and center
Purpose: scale the sketch to the desired text height.

Calls:
- `strokes_scale_center`

### Step 6 - Strokes to task space
Purpose: map 2D sketch strokes to 3D poses on the fitted board plane.

Calls:
- `strokes_to_taskspace`
- `viewer.setPath`

### Step 7 - IK solve
Purpose: solve the drawing path waypoint-by-waypoint in joint space.

Calls:
- `run_ik_trajectory`

Inside that pipeline, the Modern Robotics functions used include:
- `FKinSpace`
- `JacobianSpace`
- `MatrixLog6`
- `TransInv`
- `Adjoint`
- `se3ToVec`
- `MatrixExp6`
- `VecTose3`

### Step 8 - Execute drawing
Purpose: move to first waypoint, then execute the joint trajectory.

Calls:
- local helper `move_slow_quintic_limited`
- local helper `execute_trajectory_safe`

### Step 9 - Return HOME
Purpose: move the arm back to home after drawing.

## Included files
This clean repo intentionally excludes:
- single-motor test files
- 5-motor hardware files
- autosave `.asv` files
- unused duplicates
- temporary experiments not needed to run the writing workflow

## Notes
This repo uses relative paths so that it can be moved between computers more easily. Keep the folder structure intact unless you also update the path logic in the MATLAB scripts.