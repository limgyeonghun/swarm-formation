# Swarm Formation Control System

A ROS 2-based swarm formation control system for autonomous drones/rovers with real-time trajectory optimization and collision avoidance.

## 🏗️ Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd swarm-formation
   ```

2. **Install dependencies**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-rclcpp ros-humble-nav-msgs ros-humble-visualization-msgs
   sudo apt install libeigen3-dev libomp-dev
   ```

3. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
  
### Simulation Mode
```bash
# Launch with drone ID 1 in simulation mode
ros2 launch path_manager path_manager.launch.py drone_id:=1 real:=false rviz_simulation:=true
```

### Real Hardware Mode
```bash
# Launch with drone ID 2 and real hardware
ros2 launch path_manager path_manager.launch.py drone_id:=2 real:=true

# With auto-detected serial port
ros2 launch path_manager path_manager.launch.py drone_id:=2 real:=true jfi_port:=auto

# With specific serial port
ros2 launch path_manager path_manager.launch.py drone_id:=2 real:=true jfi_port:=/dev/ttyUSB1
```

### Examples

**Single drone simulation**:
```bash
ros2 launch path_manager path_manager.launch.py drone_id:=0 real:=false rviz_simulation:=true
```

**Real hardware with auto port detection**:
```bash
ros2 launch path_manager path_manager.launch.py drone_id:=1 real:=true jfi_port:=auto
```

**Custom serial port**:
```bash
ros2 launch path_manager path_manager.launch.py drone_id:=2 real:=true jfi_port:=/dev/ttyUSB1 jfi_baud_rate:=57600
```
