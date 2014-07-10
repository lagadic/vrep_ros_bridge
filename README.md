# vrep_ros_bridge

V-Rep ROS Bridge is a plugin for V-Rep developed by the Inria
<a href="http://www.irisa.fr/lagadic" target="_parent">Lagadic</a> team located at <a href="http://www.inria.fr/rennes" target="_parent">Inria Rennes</a>.

The main application of the plugin is to provide a communication interface between V-Rep and (ROS). The aim is to control the V-Rep simulation externally using ROS messages and ROS services.

V-Rep is a General purpose 3D robot simulator with integrated development environment developed by <a href="http://www.coppeliarobotics.com/" target="_parent">Coppelia Robotics</a>. Sensors, mechanisms, robots and whole systems can be modelled and simulated in various ways.

You will find the documentation <a href="http://wiki.ros.org/vrep_ros_bridge?distro=hydro" target="_parent">here</a>.


##Installation 


Note: The Ubuntu version used is 13.04. It works also with 12.04 and 14.04.
* <a href="#instROS_sec" target="_parent"> Install ROS Hydro</a> 
* <a href="#instvrep_sec" target="_parent"> Install V-Rep</a>
* <a href="#instplug" target="_parent"> Install Plugin</a>



###Installation ROS Hydro 


Follow instructions you find in this <a href="http://wiki.ros.org/hydro/Installation/Ubuntu" target="_parent">page</a>.

* Point 1.2 : choose instructions for Ubuntu 13.04 (Raring).

* Point 1.4 : Desktop-Full Install: (Recommended).

* When you configure the ROS Environment, choose catkin.

* Complete the tutorial <a href="http://wiki.ros.org/ROS/Tutorials" target="_parent">page</a>. It is mandatory to follow the tutorial number 1 "Installing and Configuring Your ROS Environment".

Now we have installed ROS and we have created our workspace.

###Installation ROS Indigo


As for Hydro, follow the instructions in this <a href="http://wiki.ros.org/indigo/Installation/Ubuntu" target="_parent">page</a>. 


###Installation V-Rep

* Go in <a href="http://www.coppeliarobotics.com/" target="_parent">http://www.coppeliarobotics.com/ </a>.
* Check the <a href="http://www.coppeliarobotics.com/helpFiles/en/licensing.htm" target="_parent">licensing</a> page and download the suitable V-REP version for you.
* To run it go to the folder of V-rep via terminal and type
`./vrep.sh`
* Follow <a href="http://www.coppeliarobotics.com/helpFiles/en/rosTutorial.htm" target="_parent"> this </a> tutorial.
</p>


###Installation Plugin 


* Go in the src folder of your catkin workspace in catkin_ws/src via terminal
* Download the plugin from GIT typing:

	`git clone https://github.com/lagadic/vrep_ros_bridge.git`

* Checkout the right branch depening on your ROS version (Hydro or Indigo)

        `git checkout origin/hydro-devel`
 or

        `git checkout origin/indigo-devel` 


* Add the file CATKIN_IGNORE in the sub-plugin folder that you don't need (if you don't have Telekyb installed add it in the folder imu_handler and quadrotor_tk_handler)

	`touch CATKIN_IGNORE`

* Open the file bashrc:
	` gedit ~/.bashrc`
and in the end of the file add:

`export VREP_ROOT_DIR=/ChangeWithyourPathToVrep/ `

`export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${VREP_ROOT_DIR}/programming/ros_stacks`
 

* Go in your catkin_workspace and build it with 

	`catkin_make --pkg vrep_ros_bridge --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo `

* In the folder catkin_ws/devel/lib/ you will find the main library (libv_repExtRosBridge.so) and the others libraries (libcamera_handler.so, libmanipulator_handler.so, libquadrotor_handler.so, librigid_body_handler.so ). 
* The file libv_repExtRosBridge.so has to be in the V-Rep installation folder in order to be loaded. What we will do is to create a symbolic link to it. Go via terminal to the installation folder of V-Rep and type:

	`ln -s /YOUR_CATKIN_WS_PATH/devel/lib/libv_repExtRosBridge.so`

where `/YOUR_CATKIN_WS_PATH` is your actual path to reach your workspace.

* In order to test if the installation was succesfull go  <a href="http://wiki.ros.org/vrep_ros_bridge#Installation_test" target="_parent"> here </a> 


# How to use the plugin

You will find a guide to use the plugin in the relative documentation. To create the documentation go via terminal in the source code of the package:
`cd src/vrep_ros_bridge/`
and use rosdoc_lite to create the documentation:

`rosdoc_lite vrep_ros_plugin`

This will create a the documentation in the folder `src/vrep_ros_bridge/doc`. Open the file `/doc/html/index.html` in your browser to see it. The documentation is under construction.
 
