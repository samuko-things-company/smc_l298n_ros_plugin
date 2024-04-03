# smc_ros_hw_plugin
This is a child project of the Samuko Motor Control (**`smc`**) project. This is to be used with **Ubuntu 22.04 (ros2 humble)** in your linux-based microcomputer **ROS2** mobile robotic project (as it depends on the libserial-dev linux package) to communicate with the **`smc_l298n_pid_driver module`** in order to send target angular velocities to the motors or receive the motor's angular velocity and angular position, after successful velocity PID setup with the [**`smc_app`**](https://github.com/samuko-things-company/smc_app).

> **NOTE:** should be used with your microcomputer robotics project running on linux Ubuntu 22.04 [ROS2 Humble] (e.g Raspberry Pi, PC, etc.)


## How to Use the Package
- ensure you've already set up your microcomputer or PC system with ros2-humble with colcon and your ros workspace also setup

- install the libserial-dev package on your linux machine
  > sudo apt-get update
  >
  > sudo apt install libserial-dev

- In the src/ folder of your ros workspace, clone the repo (or you can download and add it manually to the src/ folder)
  > ```git clone https://github.com/samuko-things-company/smc_ros_hw_plugin.git```

- build the packages with colcon (in your ros workspace root folder):
  > ```colcon build --packages-select smc_ros_hw_plugin``
  >
  > **NOTE:** the **smc_ros_hw_plugin** hardware interface will now be available for use in any project in your ros workspace.
  > 
  > *check the **`sample_smc_ros_hw_plugin_file`** in the **`smc_ros_hw_plugin`** pkg file to see the sample xacro file you can use in your own URDF controller file (for a differential drive robot)*

- check the [**`smc_diff_bot`**](https://github.com/samuko-things-company/smc_diff_bot) pkg to see a sample use case to test the driver module.

## Check connection of your driver module
- ensure the **`smc_l298n_pid_driver`** module (with the motors connected and fully set up for velocity PID) is connected to the microcomputer or PC via USB.

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  >
  > ```ls /dev/serial/by-path```
  >
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  > ```ls /dev/ttyU*```
  >
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on
