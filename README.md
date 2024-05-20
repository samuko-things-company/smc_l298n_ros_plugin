# smc_l298n_ros_plugin
This is a child project of the Samuko Motor Control (**`smc_l298n`**) project. This is to be used with a **`linux-based`** computer (or microcomputer) **`ROS2-based`** mobile robotic project (as it depends on the `libserial-dev` linux package) to communicate with the **`smc_l298n_pid_driver module`** in order to send target angular velocities to the motors or receive the motor's angular velocity and angular position, after successful velocity PID setup with the [**`smc_l298n_setup_application`**](https://github.com/samuko-things-company/smc_l298n_setup_application).

> PLEASE choose your preferred ros2 distro in the branch drop-down to see how to use it
> - [humble](https://github.com/samuko-things-company/smc_ros_hw_plugin/tree/humble)
