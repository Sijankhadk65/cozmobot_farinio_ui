# Fairino Python SDK

This is the Python SDK for controlling Fairino robots. It provides a comprehensive interface for robot control, including motion commands, I/O operations, and real-time state monitoring.

## Features

- **Robot Control**: Basic operations like mode switching, enabling/disabling, and drag teaching
- **Motion Commands**: Joint space (MoveJ), Cartesian space (MoveL, MoveC), servo control, and spline movements
- **I/O Operations**: Digital and analog I/O control
- **State Monitoring**: Real-time robot state feedback via socket connection
- **Logging**: Configurable logging with multiple output modes
- **Error Handling**: Comprehensive error codes and automatic reconnection

## Installation

### Prerequisites

- Python 3.6+
- Cython (for building the extension)
- Access to a Fairino robot controller

### Build and Install

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd fairino-python-sdk-main/linux
   ```

2. Build the extension:
   ```bash
   python setup.py build_ext --inplace
   ```

3. Install the package:
   ```bash
   pip install .
   ```

## Quick Start

```python
from fairino import Robot

# Initialize robot connection
robot = Robot.RPC(ip="192.168.58.2")

# Enable robot
robot.RobotEnable(1)

# Move to a joint position
joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot.MoveJ(joint_pos, tool=0, user=0)

# Move in Cartesian space
cart_pos = [300.0, 0.0, 200.0, 180.0, 0.0, 90.0]
robot.MoveL(cart_pos, tool=0, user=0)

# Get current robot state
state = robot.robot_state_pkg
print(f"Current position: {state.jt_cur_pos}")
```

## API Reference

## Class Reference

### RPC Class

The main robot control class that handles communication with the Fairino robot controller.

#### Initialization

**`__init__(ip="192.168.58.2")`**
- `ip`: Robot controller IP address (default: "192.168.58.2")

#### Attributes

- `ip_address`: Current IP address
- `robot`: XML-RPC client for command communication
- `robot_state_pkg`: Real-time robot state data (RobotStatePkg instance)
- `logger`: Logging instance
- `is_conect`: Connection status flag

#### Methods

##### Basic Robot Control

**`GetSDKVersion()`**
- Returns: (error_code, version_info)
- Get SDK and robot firmware version

**`GetControllerIP()`**
- Returns: (error_code, ip_address)
- Get controller IP address

**`Mode(state)`**
- `state`: 0 - automatic mode, 1 - manual mode
- Returns: error_code

**`RobotEnable(state)`**
- `state`: 0 - disable, 1 - enable
- Returns: error_code

**`DragTeachSwitch(state)`**
- `state`: 0 - exit drag teach, 1 - enter drag teach
- Returns: error_code

**`IsInDragTeach()`**
- Returns: (error_code, state) - 0 not in drag teach, 1 in drag teach

##### Motion Commands

**`StartJOG(ref, nb, dir, max_dis, vel=20.0, acc=100.0)`**
- `ref`: Reference frame (0-joint, 2-base, 4-tool, 8-user)
- `nb`: Axis number (1-6 for joints, x/y/z/rx/ry/rz for cartesian)
- `dir`: Direction (0-negative, 1-positive)
- `max_dis`: Max distance/angle
- `vel`: Velocity percentage (0-100)
- `acc`: Acceleration percentage (0-100)
- Returns: error_code

**`StopJOG(ref)`**
- `ref`: Reference frame to stop
- Returns: error_code

**`ImmStopJOG()`**
- Immediate stop all jogging
- Returns: error_code

**`MoveJ(joint_pos, tool, user, desc_pos=[0,0,0,0,0,0], vel=20.0, acc=0.0, ovl=100.0, exaxis_pos=[0,0,0,0], blendT=-1.0, offset_flag=0, offset_pos=[0,0,0,0,0,0])`**
- `joint_pos`: Target joint positions [°] (6 values)
- `tool`: Tool coordinate number (0-14)
- `user`: User coordinate number (0-14)
- `desc_pos`: Target cartesian pose [mm,°] (6 values, auto-calculated if [0,0,0,0,0,0])
- `vel`: Velocity percentage (0-100)
- `acc`: Acceleration percentage (0-100)
- `ovl`: Speed override (0-100)
- `exaxis_pos`: External axis positions (4 values)
- `blendT`: Blend time [-1 blocking, 0-500ms non-blocking]
- `offset_flag`: Offset mode (0-none, 1-work/base, 2-tool)
- `offset_pos`: Offset position [mm,°] (6 values)
- Returns: error_code

**`MoveL(desc_pos, tool, user, joint_pos=[0,0,0,0,0,0], vel=20.0, acc=0.0, ovl=100.0, blendR=-1.0, blendMode=0, exaxis_pos=[0,0,0,0], search=0, offset_flag=0, offset_pos=[0,0,0,0,0,0], oacc=100.0, config=-1, velAccParamMode=0, overSpeedStrategy=0, speedPercent=10)`**
- `desc_pos`: Target cartesian pose [mm,°] (6 values)
- `tool`: Tool coordinate number (0-14)
- `user`: User coordinate number (0-14)
- `joint_pos`: Target joint positions [°] (6 values, auto-calculated if [0,0,0,0,0,0])
- `vel`: Velocity percentage (0-100) or speed (mm/s) based on velAccParamMode
- `acc`: Acceleration percentage (0-100) or acceleration (mm/s²)
- `ovl`: Speed override (0-100)
- `blendR`: Blend radius [-1 blocking, 0-1000mm non-blocking]
- `blendMode`: Blend mode (0-inner tangent, 1-corner)
- `exaxis_pos`: External axis positions (4 values)
- `search`: Welding search (0-no, 1-yes)
- `offset_flag`: Offset mode (0-none, 1-work/base, 2-tool)
- `offset_pos`: Offset position [mm,°] (6 values)
- `oacc`: Acceleration override (0-100)
- `config`: IK config (-1-current, 0-7-specific)
- `velAccParamMode`: Parameter mode (0-percentage, 1-physical)
- `overSpeedStrategy`: Overspeed handling (0-off, 1-standard, 2-error, 3-adaptive)
- `speedPercent`: Allowed speed reduction (0-100)
- Returns: error_code

**`MoveC(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, ...)`**
- Similar parameters as MoveL but for circular motion with path point and target point
- Returns: error_code

**`Circle(desc_pos_p, tool_p, user_p, desc_pos_t, tool_t, user_t, ...)`**
- Full circle motion
- Returns: error_code

**`NewSpiral(desc_pos, tool, user, param, ...)`**
- Spiral motion
- `param`: [circle_num, circle_angle, rad_init, rad_add, rotaxis_add, rot_direction, velAccMode]
- Returns: error_code

**`ServoMoveStart()` / `ServoMoveEnd()`**
- Start/end servo motion mode
- Returns: error_code

**`ServoJ(joint_pos, axisPos, acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0, id=0)`**
- Joint servo motion
- `joint_pos`: Target joint positions [°] (6 values)
- `axisPos`: External axis positions (variable length)
- `cmdT`: Command period (s)
- Returns: error_code

**`ServoCart(mode, desc_pos, exaxis, pos_gain=[1,1,1,1,1,1], acc=0.0, vel=0.0, cmdT=0.008, filterT=0.0, gain=0.0)`**
- Cartesian servo motion
- `mode`: 0-absolute base, 1-incremental base, 2-incremental tool
- `desc_pos`: Target/incremental pose [mm,°] (6 values)
- `exaxis`: External axis positions (variable length)
- `pos_gain`: Position gains (6 values)
- Returns: error_code

**`ServoJTStart()` / `ServoJT(torque, interval, ...)` / `ServoJTEnd()`**
- Joint torque control
- `torque`: Joint torques [Nm] (6 values)
- `interval`: Control interval (s)
- Returns: error_code

**`MoveCart(desc_pos, tool, user, vel=20.0, acc=0.0, ovl=100.0, blendT=-1.0, config=-1)`**
- Cartesian point-to-point motion
- Returns: error_code

**`SplineStart()` / `SplinePTP(...)` / `SplineEnd()`**
- Spline motion commands
- Returns: error_code

**`NewSplineStart(type, averageTime=2000)` / `NewSplinePoint(...)` / `NewSplineEnd()`**
- New spline motion
- `type`: 0-arc transition, 1-point path
- Returns: error_code

**`StopMotion()` / `PauseMotion()` / `ResumeMotion()`**
- Motion control
- Returns: error_code

##### I/O Operations

**`SetDO(index, status)`**
- `index`: Output index
- `status`: 0-off, 1-on
- Returns: error_code

**`GetDO(index)`**
- `index`: Output index
- Returns: (error_code, status)

**`GetDI(index)`**
- `index`: Input index
- Returns: (error_code, status)

**`SetAO(index, value)` / `GetAO(index)`**
- Analog output operations
- `value`: Analog value (0-10000)

**`GetAI(index)`**
- Analog input read
- Returns: (error_code, value)

##### Coordinate Systems

**`SetToolCoord(tool_num, coord, type=0)`**
- Set tool coordinate
- `tool_num`: Tool number (0-14)
- `coord`: Coordinate values [mm,°] (6 values)
- `type`: 0-flange, 1-tcp
- Returns: error_code

**`SetWObjCoord(user_num, coord)`**
- Set work object coordinate
- `user_num`: User coordinate number (0-14)
- `coord`: Coordinate values [mm,°] (6 values)
- Returns: error_code

##### Logging

**`setup_logging(output_model=1, file_path="", file_num=5)`**
- Configure logging
- `output_model`: 0-file, 1-buffered, 2-queued
- `file_path`: Log file path
- `file_num`: Backup count
- Returns: error_code

**`set_log_level(lvl)`**
- `lvl`: 1-ERROR, 2-WARNING, 3-INFO, 4-DEBUG
- Returns: log_level

**`log_debug(message)` / `log_info(message)` / `log_warning(message)` / `log_error(message)`**
- Log messages at different levels

##### Kinematics

**`GetForwardKin(joint_pos)`**
- Forward kinematics
- `joint_pos`: Joint positions [°] (6 values)
- Returns: (error_code, pose) - pose [mm,°] (6 values)

**`GetInverseKin(tool, desc_pos, config=-1)`**
- Inverse kinematics
- `tool`: Tool number
- `desc_pos`: Target pose [mm,°] (6 values)
- `config`: IK config (-1-current, 0-7-specific)
- Returns: (error_code, joint_pos) - joint_pos [°] (6 values)

### Data Structures

#### RobotStatePkg (Structure)

Real-time robot state data structure.

**Fields:**
- `frame_head`: Frame header (0x5A5A)
- `frame_cnt`: Frame counter
- `data_len`: Data length
- `program_state`: Program state (1-stop, 2-run, 3-pause)
- `robot_state`: Robot state (1-stop, 2-run, 3-pause, 4-drag)
- `main_code`: Main error code
- `sub_code`: Sub error code
- `robot_mode`: Robot mode (0-auto, 1-manual)
- `jt_cur_pos`: Current joint positions [°] (6 floats)
- `tl_cur_pos`: Current tool pose [mm,°] (6 floats)
- `flange_cur_pos`: Current flange pose [mm,°] (6 floats)
- `actual_qd`: Actual joint velocities [°/s] (6 floats)
- `actual_qdd`: Actual joint accelerations [°/s²] (6 floats)
- `target_TCP_CmpSpeed`: Target TCP composite speed (2 floats)
- `target_TCP_Speed`: Target TCP speed [mm/s, °/s] (6 floats)
- `actual_TCP_CmpSpeed`: Actual TCP composite speed (2 floats)
- `actual_TCP_Speed`: Actual TCP speed [mm/s, °/s] (6 floats)
- `jt_cur_tor`: Current joint torques [Nm] (6 floats)
- `tool`: Current tool number
- `user`: Current user coordinate number
- `cl_dgt_output_h`: Control box digital outputs 15-8
- `cl_dgt_output_l`: Control box digital outputs 7-0
- `tl_dgt_output_l`: Tool digital outputs 7-0 (bits 0-1 valid)
- `cl_dgt_input_h`: Control box digital inputs 15-8
- `cl_dgt_input_l`: Control box digital inputs 7-0
- `tl_dgt_input_l`: Tool digital inputs 7-0 (bits 0-1 valid)
- `cl_analog_input`: Control box analog inputs (2 uint16)
- `tl_anglog_input`: Tool analog input (uint16)
- `ft_sensor_raw_data`: Force/torque sensor raw data (6 floats)
- `ft_sensor_data`: Force/torque sensor processed data (6 floats)
- `ft_sensor_active`: FT sensor active flag (0-reset, 1-active)
- `EmergencyStop`: Emergency stop flag
- `motion_done`: Motion completion flag
- `gripper_motiondone`: Gripper motion done flag
- `mc_queue_len`: Motion command queue length
- `collisionState`: Collision state (1-collision, 0-no collision)
- `trajectory_pnum`: Trajectory point number
- `safety_stop0_state`: Safety stop SI0 state
- `safety_stop1_state`: Safety stop SI1 state
- `gripper_fault_id`: Faulty gripper ID
- `gripper_fault`: Gripper fault code
- `gripper_active`: Gripper active status
- `gripper_position`: Gripper position
- `gripper_speed`: Gripper speed
- `gripper_current`: Gripper current
- `gripper_tmp`: Gripper temperature
- `gripper_voltage`: Gripper voltage
- `auxState`: Auxiliary state (ROBOT_AUX_STATE)
- `extAxisStatus`: External axis status (4 EXT_AXIS_STATUS)
- `extDIState`: External digital inputs (8 uint16)
- `extDOState`: External digital outputs (8 uint16)
- `extAIState`: External analog inputs (4 uint16)
- `extAOState`: External analog outputs (4 uint16)
- `rbtEnableState`: Robot enable state
- `jointDriverTorque`: Joint driver torques [Nm] (6 floats)
- `jointDriverTemperature`: Joint driver temperatures [°C] (6 floats)
- `year`, `month`, `day`, `hour`, `minute`, `second`, `millisecond`: Timestamp
- `softwareUpgradeState`: Software upgrade state
- `endLuaErrCode`: End Lua error code
- `cl_analog_output`: Control box analog outputs (2 uint16)
- `tl_analog_output`: Tool analog output (uint16)
- `gripperRotNum`: Rotating gripper turns (float)
- `gripperRotSpeed`: Rotating gripper speed % (uint8)
- `gripperRotTorque`: Rotating gripper torque % (uint8)
- `weldingBreakOffState`: Welding break-off state (WELDING_BREAKOFF_STATE)
- `jt_tgt_tor`: Target joint torques [Nm] (6 floats)
- `smartToolState`: Smart tool button state
- `wideVoltageCtrlBoxTemp`: Wide voltage control box temperature (°C)
- `wideVoltageCtrlBoxFanCurrent`: Fan current (mA)
- `toolCoord`: Tool coordinate [mm,°] (6 floats)
- `wobjCoord`: Work object coordinate [mm,°] (6 floats)
- `extoolCoord`: External tool coordinate [mm,°] (6 floats)
- `exAxisCoord`: External axis coordinate [mm,°] (6 floats)
- `load`: Payload mass (kg)
- `loadCog`: Payload center of gravity [mm] (3 floats)
- `lastServoTarget`: Last servo target [°] (6 floats)
- `servoJCmdNum`: ServoJ command count
- `check_sum`: Checksum

#### ROBOT_AUX_STATE (Structure)

Auxiliary servo state.

**Fields:**
- `servoId`: Servo ID
- `servoErrCode`: Servo error code
- `servoState`: Servo state
- `servoPos`: Servo position
- `servoVel`: Servo velocity
- `servoTorque`: Servo torque

#### EXT_AXIS_STATUS (Structure)

External axis status.

**Fields:**
- `pos`: Position
- `vel`: Velocity
- `errorCode`: Error code
- `ready`: Ready flag
- `inPos`: In position flag
- `alarm`: Alarm flag
- `flerr`: Following error flag
- `nlimit`: Negative limit flag
- `pLimit`: Positive limit flag
- `mdbsOffLine`: Modbus offline flag
- `mdbsTimeout`: Modbus timeout flag
- `homingStatus`: Homing status

#### WELDING_BREAKOFF_STATE (Structure)

Welding break-off state.

**Fields:**
- `breakOffState`: Break-off state
- `weldArcState`: Arc break-off state

### RobotError Class

Error code constants.

**Constants:**
- `ERR_SUCCESS = 0`
- `ERR_POINTTABLE_NOTFOUND = -7`
- `ERR_NOT_FOUND_LUA_FILE = -5`
- `ERR_RPC_ERROR = -4`
- `ERR_SOCKET_COM_FAILED = -2`
- `ERR_OTHER = -1`
- `ERROR_RECONN = -8`
- `ERR_SOCKET_RECV_FAILED = -16`
- `ERR_SOCKET_SEND_FAILED = -15`
- `ERR_FILE_OPEN_FAILED = -14`
- `ERR_FILE_TOO_LARGE = -13`
- `ERR_UPLOAD_FILE_ERROR = -12`
- `ERR_FILE_NAME = -11`
- `ERR_DOWN_LOAD_FILE_WRITE_FAILED = -10`
- `ERR_DOWN_LOAD_FILE_CHECK_FAILED = -9`
- `ERR_DOWN_LOAD_FILE_FAILED = -8`
- `ERR_UPLOAD_FILE_NOT_FOUND = -7`
- `ERR_SAVE_FILE_PATH_NOT_FOUND = -6`

### BufferedFileHandler Class

Custom logging handler with buffering.

**Inherits from:** `logging.handlers.RotatingFileHandler`

**Methods:**
- `__init__(filename, mode='a', maxBytes=0, backupCount=0, encoding=None, delay=False)`
- `emit(record)`: Buffered emit

### LogWriterThread Class

Thread for writing log records.

**Inherits from:** `threading.Thread`

**Methods:**
- `__init__(queue, log_handler)`
- `run()`: Process log records from queue

## Troubleshooting

### Connection Issues

1. **Cannot connect to robot**: Check IP address and network connectivity
2. **Timeout errors**: Ensure robot controller is powered on and accessible
3. **Reconnection failures**: Verify robot is in operational state

### Motion Errors

1. **Singularity errors**: Avoid joint configurations that cause singularities
2. **Out of range**: Ensure target positions are within robot workspace
3. **Safety stop**: Check safety inputs and emergency stop status

### Build Issues

1. **Cython not found**: Install Cython: `pip install cython`
2. **Compilation errors**: Ensure compatible Python version (3.6+)
3. **Import errors**: Verify the extension was built correctly

## Examples

See the `example/` directory for comprehensive usage examples including:

- Basic motion commands
- I/O operations
- Servo control
- Trajectory following
- Error handling

## License

[Add license information here]

## Support

For technical support, contact Fairino Robotics support team.