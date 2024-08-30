# Robotics Project

## Setup

### Clone this repository

```bash
git clone --recurse-submodules https://github.com/norpie/robotics-project
```

### Build the project

```bash
colcon build --symlink-install
```

### Source the workspace

> [!IMPORTANT]
> Run this command in every new terminal you open to work on this project.

```bash
source install/setup.bash
```

### Running the project

#### Launch the world

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=True
```

#### Launch the navigation stack

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

#### Launch the map saver

```bash
ros2 launch nav2_map_server map_saver_server.launch.py
```

#### Launch the project's nodes

```bash
ros2 launch project project_launch_file.launch.py
```

### Setup the TUI

```bash
python -m venv .venv
source .venv/bin/activate
# The steps above are recommended but not required
pip install -r requirements.txt
python3 tui.py
```
