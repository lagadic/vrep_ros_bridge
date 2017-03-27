# V-Rep ROS Bridge

V-Rep ROS Bridge is a plugin for V-Rep developed by the Inria
<a href="http://www.irisa.fr/lagadic" target="_parent">Lagadic</a> team located at <a href="http://www.inria.fr/rennes" target="_parent">Inria Rennes</a>.

The main application of the plugin is to provide a communication interface between V-Rep and (ROS). The aim is to control the V-Rep simulation externally using ROS messages and ROS services.

V-Rep is a General purpose 3D robot simulator with integrated development environment developed by <a href="http://www.coppeliarobotics.com/" target="_parent">Coppelia Robotics</a>. Sensors, mechanisms, robots and whole systems can be modelled and simulated in various ways.

You will find the documentation <a href="http://wiki.ros.org/vrep_ros_bridge?distro=hydro" target="_parent">here</a>.


## Installation 


Note: The Ubuntu version used is 13.04. It is tested also with 12.04 and 14.04 (with ROS Indigo).
* <a href="#ros" target="_parent"> Install ROS </a> 
* <a href="#installation-v-rep" target="_parent"> Install V-Rep</a>
* <a href="#installation-plugin" target="_parent"> Install Plugin</a>

### ROS 

#### Installation ROS Hydro 


Follow instructions you find in this <a href="http://wiki.ros.org/ROS/Installation" target="_parent">page</a>.

* Point 1.2 : choose instructions for your version of Ubuntu (or other OS).

* Point 1.4 : Desktop-Full Install: (Recommended).

* When you configure the ROS Environment, choose catkin.

* Complete the tutorial <a href="http://wiki.ros.org/ROS/Tutorials" target="_parent">page</a>. It is mandatory to follow the tutorial number 1 "Installing and Configuring Your ROS Environment".

Now we have installed ROS and we have created our workspace (`catkin_ws`).

#### Installation ROS Indigo


As for Hydro, follow the instructions in this <a href="http://wiki.ros.org/indigo/Installation/Ubuntu" target="_parent">page</a>. 


### Installation V-Rep

* Go in <a href="http://www.coppeliarobotics.com/" target="_parent">http://www.coppeliarobotics.com/ </a>.
* Check the <a href="http://www.coppeliarobotics.com/helpFiles/en/licensing.htm" target="_parent">licensing</a> page and download the suitable V-REP version for you.
* To run it go to the folder of V-rep via terminal and type
`./vrep.sh`
* Follow <a href="http://www.coppeliarobotics.com/helpFiles/en/rosTutorial.htm" target="_parent"> this </a> tutorial (Optional).
</p>


### Installation Plugin 


* Go in the src folder of your catkin workspace in catkin_ws/src via terminal
* Download the plugin from GIT typing:

	`git clone https://github.com/lagadic/vrep_ros_bridge.git`

* Use the branch `master` if you are using ROS Indigo or ROS Jade
* If you are using Hydro:

    `git checkout hydro-devel`



* It is possible that you will not need all the sub-plugins provided by the vrep_ros_bridge. A common case is the one in which you do not need to use the TeleKyb framework (if you do not know what TeleKyb is, it is likely that you will not need it). To ignore this sub-plugin you need to add the file CATKIN_IGNORE in the sub-plugin folder that you do not need (as it is done in quadrotor_tk_handler). To do it go via terminal in the sub-plugin folder and execute: 

	`touch CATKIN_IGNORE`

* Open the file bashrc:
	` gedit ~/.bashrc`
in the end of the file add:

	`export VREP_ROOT_DIR=/ChangeWithyourPathToVrep/`

and, if you want to avoid to type the following command every time:	
	
``` 
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/path_to_catkin_ws/catkin_ws/src
source /opt/ros/indigo/setup.bash
source /path_to_catkin_ws/catkin_ws/devel/setup.bash
```

* Go in your catkin_workspace and run:

	`catkin_make `

* Now build again the pkg using the next instruction:

	`catkin_make --pkg vrep_ros_bridge --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo `

* In the folder catkin_ws/devel/lib/ you will find the main library (libv_repExtRosBridge.so) and the others libraries (libcamera_handler.so, libmanipulator_handler.so, libquadrotor_handler.so, librigid_body_handler.so ). 
* The file libv_repExtRosBridge.so has to be in the V-Rep installation folder in order to be loaded. What we will do is to create a symbolic link to it. Go via terminal to the installation folder of V-Rep and type:

	`ln -s /YOUR_CATKIN_WS_PATH/devel/lib/libv_repExtRosBridge.so`

where `/YOUR_CATKIN_WS_PATH` is your actual path to reach your workspace.

* If you are using a new version of V-REP (V3_3_1_64_Linux) we need an additional step. We need to create a link pointing to the file compiledRosPlugins/libv_repExtRos.so in the V-REP root folder. Go via terminal to the installation folder of V-Rep and type:

	`ln -s compiledRosPlugins/libv_repExtRos.so`	

* In order to test if the installation was succesfull go  <a href="http://wiki.ros.org/vrep_ros_bridge#Installation_test" target="_parent"> here </a> 

#### Update the vrep_ros_bridge according to a new V-REP release

V-REP is a software undergoing through rapid development and there can be different releases in a short period of time. <br>
If the user wants to use the vrep_ros_bridge always with the latest version of V-REP he/she has just to follow the following steps:
* Download the last version of V-REP <a href="http://www.coppeliarobotics.com/downloads.html" target="_parent"> here </a>.
</p>
* Go back to the <a href="#installation-plugin" target="_parent"> Install Plugin</a> section of this page and change the path to the one corresponding to the new version of V-REP.

Note: 
* The vrep_ros_bridge was tested up to the version 3.2.1 of V-REP released the May 4th 2015. 
* If you encounter some problems of speed in the your simulations of V-REP (e.g. jumps in the frames) you could try to change the following parameter. In V-REP version 3.2.1 there is a feature that was mistakenly disabled, which will actually accelerate visualization of your scene. If you go in Tools-> User settings, under Adjust OpenGl settings, under VBO operation, make sure that default (recommended) is selected.

# How to use the plugin

You will find a guide to use the plugin <a href="http://wiki.ros.org/vrep_ros_bridge" target="_parent"> here </a> .
