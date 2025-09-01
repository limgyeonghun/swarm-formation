# Swarm Formation Control System

A ROS 2-based swarm formation control system for autonomous drones/rovers with real-time trajectory optimization and collision avoidance.

## 🚀 Features

- **Real-time Trajectory Optimization**: L-BFGS based optimization with early exit conditions
- **Swarm Formation Control**: Multi-agent formation flying with collision avoidance
- **2D/3D Path Planning**: A* algorithm with ESDF (Explicit Signed Distance Field)
- **Hardware Integration**: JFI communication for real hardware control
- **Simulation Support**: RViz visualization for testing and debugging
- **Performance Optimized**: Compiler optimizations for Jetson Orin platform

## 📋 Prerequisites

- **ROS 2 Humble** or later
- **Ubuntu 22.04** or later
- **CMake 3.5** or later
- **Eigen3** library
- **OpenMP** (optional, for parallel processing)

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

## 🎯 Quick Start

### Jetson Orin Performance Optimization

For optimal performance on Jetson Orin platform, use the provided optimization scripts:

#### Manual Optimization
```bash
# Run performance optimization script
chmod +x scripts/jetson_optimization/optimize_jetson.sh
./scripts/jetson_optimization/optimize_jetson.sh

# Monitor performance in real-time
chmod +x scripts/jetson_optimization/monitor_performance.sh
./scripts/jetson_optimization/monitor_performance.sh
```

#### Automatic Optimization (Boot-time)
```bash
# Install systemd service for automatic optimization
sudo cp scripts/jetson_optimization/jetson-optimizer.service /etc/systemd/system/
sudo systemctl enable jetson-optimizer.service
sudo systemctl start jetson-optimizer.service

# Check service status
sudo systemctl status jetson-optimizer.service
```

#### Performance Monitoring
```bash
# Real-time performance monitoring
./scripts/jetson_optimization/monitor_performance.sh

# Check current optimization status
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
sudo nvpmodel -q
free -h
```

**📖 Detailed Guide**: See `scripts/jetson_optimization/JETSON_OPTIMIZATION.md` for complete documentation.

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

## 📁 Project Structure

```
swarm-formation/
├── src/
│   ├── path_manager/          # Main path planning and management
│   │   ├── config/           # Configuration files
│   │   │   ├── drones.yaml   # Drone configurations
│   │   │   ├── map.yaml      # Map and obstacle settings
│   │   │   └── optimizer_params.yaml # Optimization parameters
│   │   └── launch/           # Launch files
│   ├── path_planner/         # A* path planning and grid map
│   ├── path_optimizer/       # Trajectory optimization (L-BFGS)
│   ├── path_visualization/   # RViz visualization
│   ├── j_fi/                 # JFI communication for hardware
│   ├── rover_control/        # Rover control interface
│   └── swarm_graph/          # Swarm formation graph
├── scripts/
│   └── jetson_optimization/  # Jetson Orin performance optimization
│       ├── optimize_jetson.sh
│       ├── monitor_performance.sh
│       ├── jetson-optimizer.service
│       └── JETSON_OPTIMIZATION.md
```

## ⚙️ Configuration

### Drone Configuration (`drones.yaml`)
```yaml
num_drones: 1

drone_0:
  drone_id: 0
  start_point_x: 0.0
  start_point_y: 0.0
  start_point_z: 0.0
  end_point_x: 0.0
  end_point_y: 30.0
  end_point_z: 0.0
```

### Optimization Parameters (`optimizer_params.yaml`)
```yaml
optimization:
  constrain_points_perPiece: 3
  weight_obstacle: 50000.0
  weight_swarm: 50000.0
  weight_feasibility: 10000.0
  max_vel: 1.5
  max_acc: 2.0
```

### Map Configuration (`map.yaml`)
```yaml
grid_map:
  map_size_x: 20.0
  map_size_y: 20.0
  map_size_z: 0.3
  resolution: 0.1
  obstacles_inflation: 0.1
```

## 🎮 Usage

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `drone_id` | 1 | Target drone ID (0-5) |
| `real` | false | Enable real hardware mode |
| `rviz_simulation` | false | Enable RViz visualization |
| `jfi_port` | auto | JFI serial port (auto-detection) |
| `jfi_baud_rate` | 115200 | JFI serial baud rate |

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


### Common Issues

1. **Serial port not found**:
   ```bash
   # Check available ports
   ls /dev/ttyUSB* /dev/ttyACM*
   
   # Use auto-detection
   ros2 launch path_manager path_manager.launch.py jfi_port:=auto
   ```

2. **Permission denied for serial port**:
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Reboot or logout/login
   ```

3. **Performance issues**:
   - Check CPU frequency: `cat /proc/cpuinfo | grep MHz`
   - Monitor system resources: `htop`
   - Adjust optimization parameters in `optimizer_params.yaml`

### Debug Mode
Enable debug output by setting log level:
```bash
ros2 launch path_manager path_manager.launch.py --ros-args --log-level path_manager:=DEBUG
```

## 📊 System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Path Manager  │    │  Path Optimizer │    │   Path Planner  │
│                 │    │                 │    │                 │
│ • FSM Control   │◄──►│ • L-BFGS Opt    │◄──►│ • A* Algorithm  │
│ • Trajectory    │    │ • Cost Function │    │ • Grid Map      │
│ • Formation     │    │ • Constraints   │    │ • ESDF          │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  Rover Control  │    │   JFI Comm      │    │ Visualization   │
│                 │    │                 │    │                 │
│ • Hardware Ctrl │    │ • Serial Comm   │    │ • RViz Display  │
│ • Safety Check  │    │ • MAVLink       │    │ • Trajectory    │
│ • Status Monitor│    │ • System ID     │    │ • Formation     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```