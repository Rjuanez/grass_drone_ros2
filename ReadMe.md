# Setting up the project
In order to configure the project and be capable of running it there are a certain steps that must be taken into account.
## Building the project
Use `colcon build` to build the whole project, or `colcon build --symlink-install`  In order to make the project compile automatically.

## Sourcing the project
Once the project is build it's time to add to `.bashrc` file the following `source` commands:

To source ros2 `source /opt/ros/humble/setup.bash`, to source the new project `source <path to your folder>/grass_drone_ros2/install/setup.sh`
