# vrep_ros_bridge

V-Rep ROS Bridge is a plugin for V-Rep developed by the Inria
<a href="http://www.irisa.fr/lagadic" target="_parent">Lagadic</a> team located at <a href="http://www.inria.fr/rennes" target="_parent">Inria Rennes</a>.

The main application of the plugin is to provide a communication interface between V-Rep and (ROS). The aim is to control the V-Rep simulation externally using ROS messages and ROS services.

V-Rep is a General purpose 3D robot simulator with integrated development environment developed by <a href="http://www.coppeliarobotics.com/" target="_parent">Coppelia Robotics</a>. Sensors, mechanisms, robots and whole systems can be modelled and simulated in various ways.


##Installation 


Note: The Ubuntu version used is 13.04.
* <a href="#instROS_sec" target="_parent"> Install ROS Hydro</a> 
* <a href="#instvrep_sec" target="_parent"> Install V-Rep</a>
* <a href="#instplug" target="_parent"> Install Plugin</a>



###Installation ROS Hydro 


Follow instructions you find in this <a href="http://wiki.ros.org/hydro/Installation/Ubuntu" target="_parent">page</a>.

* Point 1.2 : choose instruction for Ubuntu 13.04 (Raring).

* Point 1.4 : Desktop-Full Install: (Recommended).

* When you configure the ROS Environment, choose catkin.

* Complete the tutorial <a href="http://wiki.ros.org/ROS/Tutorials" target="_parent">page</a>. It is mandatory to follow the tutorial number 1 "Installing and Configuring Your ROS Environment".

Now we have installed ROS and we have created our workspace.


###Installation V-Rep

* Go in <a href="http://www.coppeliarobotics.com/" target="_parent">http://www.coppeliarobotics.com/ </a>  and download V-Rep.
* Extract it where you prefer
* To run it go to the folder of V-rep via terminal and type
`./vrep.sh`
* Follow this <a href="http://www.coppeliarobotics.com/" target="_parent">http://www.coppeliarobotics.com/helpFiles/en/rosTutorial.htm </a> tutorial.
</p>


###Installation Plugin 


* Go in the src folder of your catkin workspace in catkin_ws/src via terminal
* Download the plugin from GIT typing:

	`git clone https://github.com/lagadic/vrep_ros_bridge.git`

* Add the file CATKIN_IGNORE in the sub-plugin folder that you don't need (if you don't have Telekyb installed add it in the folder imu_handler and quadrotor_tk_handler)

	`touch CATKIN_IGNORE`

* Go in your catkin_workspace and build it with 

	`catkin_make --pkg vrep_ros_bridge --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo `

* In the folder vrep_ros_bridge/devel/lib/ you will find the library: libv_repExtRosBridge.so. 
* The file libv_repExtRosBridge.so has to be in the V-Rep installation folder in order to be loaded. What we will do is to create a symbolic link to it. Go via terminal to the installation folder of V-Rep and type:

	`ln -s /YOUR_CATKIN_WS_PATH/devel/lib/libv_repExtRosBridge.so`

Remember to modify `/YOUR_CATKIN_WS_PATH` with your actual path to reach your workspace.

* Type roscore in a terminal
* Now run V-Rep via terminal and check if the plugin is loaded correctly.
* If an external console will appear with the avaible plugins the plugin is working.


# How to use the plugin

You will find a guide to use the plugin in the relative documentation. To create the documentation go via terminal in the source code of the package:
`cd src/vrep_ros_bridge/`
and use rosdoc_lite to create the documentation:

`rosdoc_lite vrep_ros_plugin`

This will create a the documentation in the folder `src/vrep_ros_bridge/doc`. Open the file `/doc/html/index.html` in your browser to see it. The documentation is under construction.
 
