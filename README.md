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
# Launch with rovers in simulation mode
ros2 launch path_manager path_manager.launch.py real:=false rviz_simulation:=true enable_visualization:=true
```

### Real Hardware Mode
```bash
# Launch with rover ID 2 and real hardware
ros2 launch path_manager path_manager.launch.py drone_id:=2 real:=true record_bag:=true

```