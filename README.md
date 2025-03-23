# PhoenixMotorInterface
Repo for development of the Phoenix Motor Interface for Autonomy and Teleop using ROS

## Nodes

### Phoenix5 Node
The Phoenix 5 Node is responsible for running motors controlled with a `Talon SRX`. This node will be used for motors like Linear Actuators (and torque motors). The `Talon SRX`'s are only supported by the Phoenix 5 API. This complicates out motor interface since for non-frc purposes we cannot have both Phoenix 5 and Phoenix 6 (TalonFX) APIs running together. The Phoenix 5 API also cannot be installed as a package and instead we are using it as a Library.

#### Building Node
Due to using Phoenix 5 as a library, colcon and CMake do not automatically link the library and executable.

Start by navigating to the **root directory** of project: `pkgs`.

If this is your first time building please use `colcon build --executor parallel --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON`. Using the `--cmake-args=DCMAKE_EXPORT_COMPILE_COMMANDS` can be used to prevent error squiggles on non-standard C++ libraries (phoenix, ros, custom messages).

Otherwise, if you have already build previously run `colcon build` or `colcon build --executor parallel`. 

After compiling of code, you will need to link the Phoenix 5 library and the main executable together. To do this run the `patchelf_cmake_build_step.sh` bash script simply using `./patchelf_cmake_build_step.sh`. You should not get errors, if you do check the filepaths/filenames to make sure things are accurate.

### Phoenix 6 Node

#### Building Node


## Running Executables
For each node being ran, you will need 1 terminal for debugging purposes or push all tasks to the background (not easy to debug). Make sure to use `source install/setup.bash` to source all executables.

Every node you want to run use `ros2 run <node package> <node name>`. i.e. '_ros2 run robot_phoenix5 main_'.

Note: You do NOT need a terminal for the 'custom_types' node. This simply is just message type information.