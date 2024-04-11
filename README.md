> **NOTE:** should be used with your computer or microcomputer robotics project running on linux `Ubuntu 22.04` [ros-Humble] (e.g Raspberry Pi, PC, etc.)

## How to Use the Package
- ensure you've already set up your microcomputer or PC system with `ros-humble` with `colcon` and your `ros workspace` also setup

- install the `libserial-dev` package on your linux machine
  > ```sudo apt-get update```
  >
  > ```sudo apt install libserial-dev```

- install `rosdep` so you can install necessary ros related dependencies for the package.
  > ```sudo apt-get update```
  >
  > ```sudo apt install python3-rosdep2```
  >
  > ```rosdep update```

- In the src/ folder of your ros workspace, clone the repo (or you can download and add it manually to the src/ folder)
  > ```git clone -b humble https://github.com/samuko-things-company/smc_ros_hw_plugin.git```

- cd into the package folder (i.e `smc_ros_hw_plugin`) and run rosdep to install any necessary ros dependencies
  > ```cd smc_ros_hw_plugin```
  >
  > ```rosdep install --from-paths src --ignore-src -r -y```

- build the packages with colcon (in the root folder of your ros workspace):
  > ```colcon build --packages-select smc_ros_hw_plugin```

  > **NOTE:** the **smc_ros_hw_plugin** hardware interface will now be available for use in any project in your ros workspace.

- finally, don't forget to source your ros workspace

</br>

> *check the **`example_file`** in the **`smc_ros_hw_plugin`** pkg file to see the sample xacro file you can use in your own URDF controller file (for a differential drive robot)*
>
> AND
>
> check the [**`smc_test_bot`**](https://github.com/samuko-things-company/smc_test_bot) pkg to see a sample robot to test the smc driver module.

</br>

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
