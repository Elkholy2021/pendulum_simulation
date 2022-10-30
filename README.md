# Dynamic modeling of a simple pendulum underwater

A ros2 package for dynamic modeling of a simple pendulum fully immersed underwater with rviz visualization


![](https://github.com/Elkholy2021/pendulum_simulation/blob/main/pendulum.gif)

## Forces diagram ##
![](https://github.com/Elkholy2021/pendulum_simulation/blob/main/forces.png)
## Dynamic modeling ##
![](https://github.com/Elkholy2021/pendulum_simulation/blob/main/modeling.PNG)
For more details about the modeling, kindly see [this report](https://github.com/Elkholy2021/pendulum_simulation/blob/main/modeling_details.pdf).

## ros2 program archeticutre ##
![](https://github.com/Elkholy2021/pendulum_simulation/blob/main/architecture.png)




## Installation ##
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

## Static configuration ##
To modify the simulation parameters like mass, length, water current, drag, etc .. , kindly open the yaml file in config folder to modify them by running this command:
```bash
gedit ~/<YOUR ROS2 WORKSPACE>/src/pendulum_simulation/config/pendulum_params.yaml
```
**Note:** make sure to write the parameter values with a decimal point >> e.g. 2.0 not 2


## Starting the simulation ##
To start the simulation, kindly navigate to package directory by running this command:
```bash
cd ~/<YOUR ROS2 WORKSPACE>/src/pendulum_simulation
```
And then run the bash script ``` start_simulation.sh``` using this command:
```bash
./start_simulation.sh
```

## Configuration on the fly ##
You can also change the water current vector directrly from the terminal without building the package everytime by running the ``` water_current_pub``` and parsing the water current vector components as ros arguments by typing this command as an example:
```bash
ros2 run pendulum_simulation water_current_pub --ros-args -p "x:= 4.1" -p "y:=0.0" -p "z:=-2.1"
```
Also if you want to add some disturbances to the water current you can add the ros-arg ``` dist ``` which is integer greater than or equal to zero where zero is no disturbances and high number means high fluctions. You can simply run this command as an example:
```bash
ros2 run pendulum_simulation water_current_pub --ros-args -p "x:= 4.1" -p "y:=0.0" -p "z:=-2.1" -p "dist:=15"
```
If you did not provide the arguments, the node will publish the default values

If you did not run the node ``` water_current_pub``` or  the node got inturpted while running, the simulation parameters will be loaded from the yaml file. 


## References ##
- Varghese Mathai, Laura A. W. M. Loeffen, Timothy T. K. Chan, and Sander Wildeman. Dynamics of heavy and buoyant underwater pendulums. Journal of Fluid Mechanics, 862:348–363, 2019. doi:10.1017/jfm.2018.867.
- https://automaticaddison.com/how-to-load-a-urdf-file-into-rviz-ros-2/

