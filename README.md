# Dynamic modeling of a simple pendulum underwater
A ros2 package for dynamic modeling of a simple pendulum underwater with rviz visualization

# Installation #
Before you install the package, make sure that you have the joint state publisher and xacro packeges installed on your ros2 system by typing the following:
```bash
sudo apt install ros-<ros2-distro>-joint-state-publisher
```
```bash
sudo apt install ros-<ros2-distro>-xacro
```

After that, navigate to your ros2 workspace folder and then to the src folder by running this command:

```bash
cd ~/<YOUR ROS2 WORKSPACE>/src
```

Then, run this command:

```bash
git clone https://github.com/Elkholy2021/Dynamic-modeling-of-a-simple-pendulum-undrerwater.git
```

# Configuration #
To modify the simulation parameters, kindly open the yaml file in config folder to modify them by running this command:
```bash
gedit ~/<YOUR ROS2 WORKSPACE>/src/pendulum_simulation/config/pendulum_params.yaml
```
**Note:** make sure to write the parameter values with a decimal point >> e.g. 2.0 not 2


# Starting the simulation #
To start the simulation, kindly navigate to package directory by running this command:
```bash
cd ~/<YOUR ROS2 WORKSPACE>/src/pendulum_simulation
```
And then run the bash script ``` start_simulation.sh``` using this command:
```bash
./start_simulation.sh
```

