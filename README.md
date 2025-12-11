# ReactorX200 - Robotic Arm Control

Complete control system for the ReactorX200 robotic arm with support for MuJoCo simulation, direct physical control, and digital twins.

## 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Project Architecture](#project-architecture)
- [Main Classes](#main-classes)
- [Enumerations](#enumerations)
- [Usage Examples](#usage-examples)
- [API Reference](#api-reference)

## 🔧 Installation

### Prerequisites

- Python 3.8+
- MuJoCo 3.3.0
- MuJoCo Binaries version 3.2.7 (downloadable from [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases))
- Dynamixel SDK (for physical control)

### Installing Dependencies

```bash
pip install -r mujoco/requirements.txt
```

**Main dependencies:**
- `mujoco==3.3.0` - Physics simulator
- `dynamixel-sdk==3.7.31` - Dynamixel servo control
- `numpy==2.2.4` - Numerical computing
- `pyserial==3.5` - Serial communication

## 🚀 Quick Start

### Create a Robot Instance

```python
from mujoco.reactorx200 import ReactorX200, ExecutionType
from mujoco.manipulatorarm import Joint

# Simulation
robot = ReactorX200(exec_type=ExecutionType.Simulated)

# Physical Control (requires COM port)
robot = ReactorX200(exec_type=ExecutionType.Physical, device_name='COM5')

# Digital Twin (simulation + physical simultaneously)
robot = ReactorX200(exec_type=ExecutionType.DigitalTwin, device_name='COM5')
```

### Basic Operations

```python
# Move all joints to home position
robot.move_joints_to_home()

# Enable torque on all joints
robot.enable_joints_torques()

# Set movement velocity (RPM)
robot.set_joint_velocity(Joint.Shoulder, 15)

# Move to specific position (degrees)
robot.set_joint_position(Joint.Shoulder, 45)

# Get current position
position = robot.get_joint_position(Joint.Shoulder)

# Get force/torque
force = robot.get_joint_force(Joint.Shoulder)

# Close connection
robot.close()
```

## 📐 Project Architecture

```
reactorx200/
├── mujoco/
│   ├── reactorx200.py           # Main orchestration class
│   ├── mujocoreactorx200.py     # MuJoCo simulation implementation
│   ├── trossenreactorx200.py    # Trossen physical control implementation
│   ├── manipulatorarm.py         # Abstract base class
│   ├── servo.py                  # Servo control class
│   ├── mujococontroller.py       # Simulation controller
│   ├── dynamixelcontroller.py    # Dynamixel controller
│   ├── controller.py             # Base controller class
│   ├── errors.py                 # Custom exceptions
│   ├── requirements.txt           # Python dependencies
│   └── model/                    # 3D models and XML files
│       └── reactorx200/
│           ├── reactorx200.xml
│           ├── model.xml
│           ├── model_link.xml
│           └── assets/
├── utils/
│   ├── add_materials_to_urdf.py  # URDF tools
│   └── fix_urdf.py
└── README.md
```

## 🎯 Main Classes

### `ReactorX200`

Main class that acts as an orchestrator, allowing unified control of the robot in different modes.

**Constructor:**
```python
ReactorX200(exec_type: ExecutionType = ExecutionType.Simulated, 
            device_name: str = None)
```

**Parameters:**
- `exec_type`: Execution type (Physical, Simulated, DigitalTwin)
- `device_name`: COM port for physical robot (e.g., 'COM5')

**Main Methods:**

| Method | Description | Parameters | Returns |
|--------|-------------|-----------|---------|
| `move_joint_to_home(joint)` | Move joint to home position | `Joint` | - |
| `move_joints_to_home()` | Move all joints to home | - | - |
| `enable_joint_torque(joint)` | Enable torque on a joint | `Joint` | - |
| `enable_joints_torques()` | Enable torque on all joints | - | - |
| `disable_joint_torque(joint)` | Disable joint torque | `Joint` | - |
| `disable_joints_torques()` | Disable torque on all joints | - | - |
| `set_joint_position(joint, pos)` | Set position (degrees) | `Joint, float` | - |
| `set_joints_positions(positions)` | Set positions | `list[float]` | - |
| `get_joint_position(joint)` | Get position | `Joint` | `float` |
| `get_joints_positions()` | Get all positions | - | `list[float]` |
| `set_joint_velocity(joint, rpm)` | Set velocity (RPM) | `Joint, float` | - |
| `set_joints_velocities(velocities)` | Set velocities | `list[float]` | - |
| `get_joint_velocity(joint)` | Get velocity | `Joint` | `float` |
| `get_joints_velocities()` | Get all velocities | - | `list[float]` |
| `get_joint_force(joint)` | Get force/torque | `Joint` | `float` |
| `get_joints_forces()` | Get all forces | - | `list[float]` |
| `get_joint_position_limits(joint)` | Get position limits | `Joint` | `list[float]` |
| `get_joint_velocity_limits(joint)` | Get velocity limits | `Joint` | `list[float]` |
| `get_joints_number()` | Get number of joints | - | `int` |
| `close()` | Close connections | - | - |

### `ManipulatorArm` (Abstract Base Class)

Base class that defines the common interface for all manipulator arms.

**Methods:**
- All methods from `ReactorX200` (which delegates to this class)

### `MuJoCoReactorX200`

Simulation implementation using MuJoCo. Inherits from `ManipulatorArm`.

**Features:**
- Realistic physics simulation
- Interactive 3D visualization
- Execution in separate threads (simulation and visualization)

**Available joints:**
1. **Waist**: ±180° with torque limits of ±8 N/m
2. **Shoulder**: -108° to 113° with limits of ±18 N/m
3. **Elbow**: -108° to 93° with limits of ±13 N/m
4. **WristAngle**: -100° to 123° with limits of ±5 N/m
5. **WristRotation**: ±180° with limits of ±5 N/m
6. **Gripper**: -30° to 60° (close to open) with limits of ±8 N

### `TrossenReactorX200`

Implementation for Trossen physical robot control. Inherits from `ManipulatorArm`.

**Features:**
- Direct communication with Dynamixel servos
- Real-time control
- Requires available COM port

**Available joints:**
1. **Waist** (Servo ID 1): ±180°
2. **Shoulder** (Servo ID 2): -108° to 113°
3. **Shadow Shoulder** (Servo ID 3): -108° to 113° (reversed)
4. **Elbow** (Servo ID 4): -108° to 93°
5. **Wrist Angle** (Servo ID 5): -100° to 123°
6. **Wrist Rotation** (Servo ID 6): ±180°
7. **Gripper** (Servo ID 7): -30° to 60°

### `Servo`

Class representing an individual servo motor with unit conversion.

**Features:**
- Automatic conversion between system and application units
- Configurable safety limits
- Reverse mode support

**Constructor:**
```python
Servo(controller, servo_id, pos_sys_range, pos_app_range, 
      vel_sys_range, vel_app_range, tor_sys_range, tor_app_range,
      position_limits, velocity_limits, home_position, 
      safe_velocity, reverse_mode)
```

### Controllers

#### `MuJoCoController`

Handles MuJoCo simulation in separate threads.

**Features:**
- Independent simulation thread
- Independent visualization thread
- Thread-safe synchronization with locks

#### `DynamixelController`

Communicates with Dynamixel servos via serial port.

**Features:**
- Parallel control of multiple servos
- Servo register read/write operations
- Communication error handling

## 📌 Enumerations

### `ExecutionType`

```python
class ExecutionType(Enum):
    Physical = 0          # Physical robot control
    Simulated = 1         # MuJoCo simulation only
    DigitalTwin = 2       # Simulation + physical simultaneously
```

### `Joint`

```python
class Joint(Enum):
    Waist = 0
    Shoulder = 1
    Elbow = 2
    WristAngle = 3
    WristRotation = 4
    Gripper = 5
```

## 💡 Usage Examples

### Example 1: Simple Simulation

```python
from mujoco.reactorx200 import ReactorX200, ExecutionType
from mujoco.manipulatorarm import Joint
import time

# Create simulated robot
robot = ReactorX200(exec_type=ExecutionType.Simulated)

try:
    # Move to home position
    robot.move_joints_to_home()
    time.sleep(1)
    
    # Enable torque
    robot.enable_joints_torques()
    
    # Set movement velocity
    robot.set_joints_velocities([15] * 6)  # 15 RPM for all
    
    # Move shoulder to 45 degrees
    robot.set_joint_position(Joint.Shoulder, 45)
    time.sleep(2)
    
    # Get current position
    pos = robot.get_joint_position(Joint.Shoulder)
    print(f"Current position: {pos:.2f}°")
    
finally:
    robot.close()
```

### Example 2: Physical Robot Control

```python
from mujoco.reactorx200 import ReactorX200, ExecutionType
from mujoco.manipulatorarm import Joint
import time

# Control physical robot on COM5
robot = ReactorX200(exec_type=ExecutionType.Physical, device_name='COM5')

try:
    # Check status
    positions = robot.get_joints_positions()
    print(f"Positions: {positions}")
    
    # Move the arm
    robot.enable_joints_torques()
    robot.set_joint_position(Joint.Shoulder, 30)
    time.sleep(1)
    
    # Read force
    force = robot.get_joint_force(Joint.Shoulder)
    print(f"Applied force: {force:.1f}%")
    
finally:
    robot.disable_joints_torques()
    robot.close()
```

### Example 3: Digital Twin (Simulation + Physical)

```python
from mujoco.reactorx200 import ReactorX200, ExecutionType
from mujoco.manipulatorarm import Joint
import time

# Digital twin: run simulation and physical control in parallel
robot = ReactorX200(exec_type=ExecutionType.DigitalTwin, device_name='COM5')

try:
    # Both interfaces (simulated and physical) receive commands
    robot.move_joints_to_home()
    robot.enable_joints_torques()
    
    # Test movement
    robot.set_joint_position(Joint.Elbow, 30)
    time.sleep(2)
    
    # Readings come from physical robot (robots[0])
    pos_physical = robot.get_joint_position(Joint.Elbow)
    print(f"Position (physical): {pos_physical:.2f}°")
    
finally:
    robot.disable_joints_torques()
    robot.close()
```

### Example 4: Gripper Control

```python
from mujoco.reactorx200 import ReactorX200, ExecutionType
from mujoco.manipulatorarm import Joint
import time

robot = ReactorX200(exec_type=ExecutionType.Simulated)

try:
    robot.enable_joints_torques()
    robot.set_joint_velocity(Joint.Gripper, 20)
    
    # Get gripper position limits
    limits = robot.get_joint_position_limits(Joint.Gripper)
    print(f"Gripper limits: {limits}")  # [-30, 60]
    
    # Open gripper (60 degrees)
    print("Opening gripper...")
    robot.set_joint_position(Joint.Gripper, limits[1])
    time.sleep(2)
    
    # Close gripper (-30 degrees)
    print("Closing gripper...")
    robot.set_joint_position(Joint.Gripper, limits[0])
    time.sleep(2)
    
finally:
    robot.close()
```

### Example 5: Programmed Movement Sequence

```python
from mujoco.reactorx200 import ReactorX200, ExecutionType
from mujoco.manipulatorarm import Joint
import time

robot = ReactorX200(exec_type=ExecutionType.Simulated)

def move_to_position(robot, positions, velocity=15, wait_time=2):
    """Helper to move all joints to specific positions"""
    robot.set_joints_velocities([velocity] * robot.get_joints_number())
    robot.set_joints_positions(positions)
    time.sleep(wait_time)

try:
    robot.enable_joints_torques()
    robot.move_joints_to_home()
    time.sleep(1)
    
    # Position 1: Extended arm
    move_to_position(robot, [0, 0, 0, 0, 0, 0])
    print("Position 1: Extended arm")
    
    # Position 2: Flexed arm
    move_to_position(robot, [0, 45, 30, 0, 0, 0])
    print("Position 2: Flexed arm")
    
    # Position 3: Raised arm
    move_to_position(robot, [0, 90, -45, 0, 0, 0])
    print("Position 3: Raised arm")
    
    # Return to home
    robot.move_joints_to_home()
    
finally:
    robot.close()
```

## 📖 API Reference

### Value Ranges

#### Position
- Application range: -180° to 179.91° (degrees)
- Each joint has specific limits (see joint specifications)

#### Velocity
- Range: 0.229 to 61 RPM
- Default safe value: 10 RPM

#### Force/Torque
- Application range: -100% to 100%
- Varies by specific joint

### Unit Conventions

- **Positions**: Degrees (°)
- **Velocities**: RPM (Revolutions Per Minute)
- **Forces**: Percentage (%) or N/m depending on context
- **Time**: Seconds

### Important Notes

1. **Initialization**: Always call `move_joints_to_home()` at startup
2. **Safety**: Use low velocities (10-20 RPM) during testing
3. **Cleanup**: Always call `close()` when done
4. **Thread-Safety**: Code handles synchronization internally with locks
5. **DigitalTwin Mode**: Readings come from physical robot, not simulation

### Error Handling

```python
from mujoco.errors import *  # Import exceptions if available

try:
    robot = ReactorX200(exec_type=ExecutionType.Physical, device_name='COM5')
    # ... operations ...
except ValueError as e:
    print(f"Value error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    robot.close()
```

## 🔗 Related Files

- 3D Models: `mujoco/model/reactorx200/`
- Utilities: `utils/` (URDF processing tools)
- Documentation: This README

---

**Project Status**: Work in Progress (WiP)
**Last Updated**: December 2025
