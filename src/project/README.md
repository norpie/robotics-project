# Patrol

![Video](https://private-user-images.githubusercontent.com/46564751/276301531-8ab175a8-6238-4724-aed2-f916e1d91a77.mp4?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTEiLCJleHAiOjE2OTc2NDI3MDUsIm5iZiI6MTY5NzY0MjQwNSwicGF0aCI6Ii80NjU2NDc1MS8yNzYzMDE1MzEtOGFiMTc1YTgtNjIzOC00NzI0LWFlZDItZjkxNmUxZDkxYTc3Lm1wND9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFJV05KWUFYNENTVkVINTNBJTJGMjAyMzEwMTglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjMxMDE4VDE1MjAwNVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTBiMGE1ZjU1NmFiOTI3YTNmNTNjNGI4MzVmMzBkOGY3MzU4OTRiZDE0ZDExMDZmMDNjMGQyYjQxYTIwODQ2NTAmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.SPA_MZNAjAVQSUC_PI_fP-Zbzdxp53uNCrwg0KZD3LQ)
![Alternate](https://github.com/norpie/2023_RoboticsAi2TI_kuosmanen_konsta/issues/1#issuecomment-1768654996)

## Usage

```bash
cd /your/workspace/location                                 # navigate to workspace
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py    # launch a new gazebo instance
colcon build --packages-select project_pkg                   # build the package
source install/setup.bash                                   # source the env
ros2 launch project_pkg project_pkg_launch_file.launch.py     # launch the package
```

## Implementation

If there are any objects at -25° to 25° close in front of the turtle, rotate counter-clockwise. Else keep going forwards.
