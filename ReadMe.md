# README - Grass Drone ROS2

## Setting Up the Project
To configure the project and ensure it runs correctly, follow these steps:

### Prerequisites
Before building the project, ensure you have the following installed:
- **ROS 2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **Colcon** (ROS 2 build tool)
- **Required dependencies** (install them using `rosdep` if necessary)

### Installing Dependencies
Run the following command to install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Working with Git
Before starting any work on the project, it is crucial to ensure you have the latest version of the repository. Always follow these steps:
1. **Pull the latest changes** to ensure your local repository is up to date:
   ```bash
   git pull
   ```
2. **Check the current status** of your working directory to see if there are any changes:
   ```bash
   git status
   ```
3. **Compare your local changes** before committing to review modifications:
   ```bash
   git diff
   ```

## Uploding to GitHub (basic workflow)
1. **Check the lastest changes and the branch you are at** to ensure what you've done:
   ```bash
   git status
   ```
1. **If needed change your branch** to ensure you are on the desired one:
   ```bash
   git checkout <name_of_the_branch>
   ```
2. **Add the desired files to the commit**:
   ```bash
   git add <name_of_file>
   ```
3. **Check the additions** to ensure what you're going to upload:
   ```bash
   git status
   ```
4. **Commit your changes**:
   ```bash
   git commit -m "Description of changes"
   ```
5. **Push changes to the desired branch**:
   ```bash
   git push origin <name_of_the_branch>
   ```

## Building the Project
To compile the project, use the following command:
```bash
colcon build
```
For symlink installation (useful for development), run:
```bash
colcon build --symlink-install
```

## Sourcing the Project
Once the project is built, you need to source the necessary setup files.

### Sourcing ROS 2
Add the following to your `.bashrc` file to ensure ROS 2 is always sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Sourcing the Project
To source the project, add the following line to your `.bashrc` file, replacing `<path-to-your-folder>` with your actual project directory:
```bash
source <path-to-your-folder>/grass_drone_ros2/install/setup.sh
```
Alternatively, you can manually source it for each session:
```bash
source install/setup.sh
```

## Creating a New Package
To create a new ROS 2 package using `ros2 pkg create`, use one of the following commands depending on your build system:

**It's recommended to use ament_cmake even if you are going to use python code**

For a Python-based package:
```bash
ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs my_package
```
For a CMake-based package:
```bash
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs my_package
```
This will create a package named `my_package` with the specified dependencies.



### Typical Dependencies
Common dependencies for ROS 2 packages include:
- `rclpy`: Core ROS 2 Python client library
- `rclcpp`: Core ROS 2 C++ client library
- `std_msgs`: Standard message types
- `sensor_msgs`: Messages for sensor data
- `geometry_msgs`: Messages for geometric data
- `nav_msgs`: Messages for navigation-related data

### Setting Up the Package
Once the package is created, navigate into the package directory and source the environment:
```bash
cd my_package
colcon build --symlink-install
source install/setup.sh
```

## Running the Project
To launch the project, use the appropriate ROS 2 launch file. Example:
```bash
ros2 launch <package_name> <launch_file>.launch.py
```

### Launching the Simulation
To launch the Gazebo simulation, use the following command:
```bash
ros2 launch grass_drone_bringup gazebo.launch.py
```



