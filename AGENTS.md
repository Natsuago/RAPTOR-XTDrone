# Repository Guidelines

## Project Structure & Module Organization
This workspace combines three main codebases:
- `XTDrone/`: simulation and autonomy packages (communication, control, coordination, motion planning, sensing, Gazebo assets under `sitl_config/`).
- `PX4_Firmware/`: autopilot firmware (C/C++) with its own `src/`, `Tools/`, `test/`, and build system.
- `catkin_ws/`: ROS1 catkin workspace (currently includes `src/gazebo_ros_pkgs` plus generated `build/` and `devel/`).
- `docs/paper/`: reference papers (`RAPTOR.pdf`, `XTDrone.pdf`).

Keep edits in source directories; do not commit generated artifacts from `catkin_ws/build`, `catkin_ws/devel`, or `PX4_Firmware/build`.

## Build, Test, and Development Commands
Use repo-local commands so components stay independent:
```bash
git -C XTDrone submodule update --init --recursive
git -C PX4_Firmware submodule update --init --recursive
make -C PX4_Firmware px4_sitl
make -C PX4_Firmware tests
cd catkin_ws && catkin_make && source devel/setup.bash
catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
```
Common launch entry points are in `XTDrone/sitl_config/launch/` (for example `outdoor1.launch`) and can be run with `roslaunch` after your ROS environment is sourced.

## Coding Style & Naming Conventions
- Python: 4-space indentation, `snake_case` for modules/functions, descriptive topic and node names.
- C++: follow existing local style in each package; prefer small, focused classes and ROS-friendly naming.
- Shell scripts: keep executable (`chmod +x`) and use lowercase underscore filenames.
- For PX4 formatting/linting, run:
```bash
make -C PX4_Firmware format
make -C PX4_Firmware check_format
```

## Testing Guidelines
- PX4: `make -C PX4_Firmware tests` for unit tests; `make -C PX4_Firmware rostest` for ROS/SITL integration tests.
- ROS packages: `catkin_make run_tests` in a workspace, or package-scoped commands such as:
```bash
catkin build darknet_ros --no-deps --verbose --catkin-make-args run_tests
```
- Follow existing naming (`test_*.cpp`, `*.test`) and keep tests near package `test/` directories.

## Commit & Pull Request Guidelines
Commit messages in `XTDrone` are short and imperative (`add ...`, `fix bug: ...`, `update ...`); keep that style and include a subsystem when possible (example: `communication: fix mavros reconnect check`).

For `PX4_Firmware`, follow scoped subjects seen upstream (example: `ekf2: guard velocity fusion path`).

PRs should include:
- problem statement and affected module(s),
- concise solution summary,
- exact test commands/scenarios run,
- linked issues,
- logs/screenshots when behavior changes in simulation.
