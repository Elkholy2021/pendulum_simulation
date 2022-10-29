# Dynamic modeling of a simple pendulum underrwater
A ros2 package for dynamic modeling of a simple pendulum underwater with rviz visualization

# Installation #
Before you install the package, make sure that you have the joint state publisher and xacro packeges installed on your ros2 system by typing the following:
```bash
sudo apt install ros-<ros2-distro>-joint-state-publisher
```
```bash
sudo apt install ros-<ros2-distro>-xacro
```

After that, navigate to your ros2 workspace folder and then to the src folder so that you will be in this directroy

```bash
~/<YOUR ROS2 WORKSPACE>/src
```

Run this command:

```bash
git clone https://github.com/Elkholy2021/Dynamic-modeling-of-a-simple-pendulum-undrerwater.git
```

# Configuration #
To modify the simulation parameters, kindly open this file to modifiy them:
```bash
~/<YOUR ROS2 WORKSPACE>/src/pendulum_simulation/config/pendulum_params.yaml
```
**Note:** make sure to write the parameter values with a decimal point >> e.g. 2.0 not 2


# Starting the simulation #
To start the simulation, kindly navigate to package directory:
```bash
~/<YOUR ROS2 WORKSPACE>/src/pendulum_simulation
```
And then run the bash script ``` start_simulation.sh``` using this command:
```bash
./start_simulation.sh
```

