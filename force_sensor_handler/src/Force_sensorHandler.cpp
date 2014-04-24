
#include <pluginlib/class_list_macros.h>

#include <force_sensor_handler/Force_sensorHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <geometry_msgs/WrenchStamped.h>
#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>


Force_sensorHandler::Force_sensorHandler() : GenericObjectHandler(),
    //_handleOfForceSensor(-1),
    _lastPublishedForceSensor(0.0),
    _acquisitionFrequency(-1.0)
  {
}

Force_sensorHandler::~Force_sensorHandler(){
}

unsigned int Force_sensorHandler::getObjectType() const {
    return CustomDataHeaders::FORCE_SENSOR_DATA_MAIN;
}

void Force_sensorHandler::synchronize(){


       // Remove # chars for compatibility with ROS
    _associatedObjectName = simGetObjectName(_associatedObjectID);
    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');
    _pub = _nh.advertise<geometry_msgs::WrenchStamped>(objectName+"/data", 1000);

}

void Force_sensorHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    ros::Time now = ros::Time::now();

    const simFloat currentSimulationTime = simGetSimulationTime();


    if ( ((currentSimulationTime - _lastPublishedForceSensor) >= 1.0/_acquisitionFrequency) ){


    	simFloat force[3];
    	simFloat torque[3];

    	int read_check = -1;

    	read_check = simReadForceSensor(_associatedObjectID, force, torque);

		if(read_check > 0){

			//std::cout <<"- [" << _associatedObjectName << "]: Reading data from Force Sensor" << std::endl;

			// Fill the force sensor msg
			geometry_msgs::WrenchStamped msg;
			msg.header.stamp = ros::Time::now();
			msg.wrench.force.x = force[0];
			msg.wrench.force.y = force[1];
			msg.wrench.force.z = force[2];
			msg.wrench.torque.x = torque[0];
			msg.wrench.torque.y = torque[1];
			msg.wrench.torque.z = torque[2];
			_pub.publish(msg); // Publish the Force sensor message in ROS

			_lastPublishedForceSensor = currentSimulationTime;
		}

		else
		{
			std::cout <<"- [" << _associatedObjectName << "]:  Cannot read the data from the force sensor" << std::endl;

		}


    }

}


void Force_sensorHandler::_initialize(){
    if (_initialized)
        return;

    // get some data from the main object
    std::vector<unsigned char> developerCustomData;
    getDeveloperCustomData(developerCustomData);


    // 2. From that retrieved data, try to extract sub-data with the IMU_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;
    std::stringstream ss;

    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::FORCE_SENSOR_DATA_MAIN,tempMainData)){
        _acquisitionFrequency=CAccess::pop_float(tempMainData);
        if (_acquisitionFrequency > 0.0){
            ss << "- [" << _associatedObjectName << "] Frequency publisher: " << _acquisitionFrequency << "." << std::endl;
        } else {
            ss << "- [" << _associatedObjectName << "] Frequency publisher: simulation frequency."  << std::endl;
        }
    } else {
        ss << "- [" << _associatedObjectName << "] Force Sensor frequency publisher not specified. using simulation frequency as default."  << std::endl;
    }



//    // search associated objects
//    std::vector<int> toExplore;
//    toExplore.push_back(_associatedObjectID); // We start exploration with the base of the quadrotor-tree
//    while (toExplore.size()!=0)
//    {
//        int objHandle=toExplore[toExplore.size()-1];
//        toExplore.pop_back();
//        // 1. Add this object's children to the list to explore:
//        int index=0;
//        int childHandle=simGetObjectChild(objHandle,index++);
//        while (childHandle!=-1) {
//            toExplore.push_back(childHandle);
////            std::cout << "Adding " << simGetObjectName(childHandle) << " to exploration list." << std::endl;
//            childHandle=simGetObjectChild(objHandle,index++);
//        }
//        // 2. Now check if this object has one of the tags we are looking for:
//        // a. Get all the developer data attached to this scene object (this is custom data added by the developer):
//        int buffSize=simGetObjectCustomDataLength(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);
//        if (buffSize!=0) { // Yes there is some custom data written by us (the developer with the DEVELOPER_DATA_HEADER header)
//            char* datBuff=new char[buffSize];
//            simGetObjectCustomData(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
//            std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
//            delete[] datBuff;
//            // b. From that retrieved data, try to extract sub-data with the searched tags:
//            std::vector<unsigned char> tempMainData;
//
//            if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::IMU_DATA_MASS,tempMainData)){
//                _handleOfMass=objHandle;
//                simGetObjectFloatParameter(_handleOfMass, 3005, &_mass);
//
//            } else if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::IMU_DATA_FORCE,tempMainData)){
//                _handleOfForceSensor=objHandle; // We found the IMU_DATA_FORCE tag. This is the Force sensor!
//                ss << "- [" << _associatedObjectName << "] Found ForceSensor. The acceleration will be computed and published." << std::endl;
//            }
//
//        }
//    }



    ConsoleHandler::printInConsole(ss);

    _lastPublishedForceSensor = -1.0;

    _initialized=true;
}

bool Force_sensorHandler::endOfSimulation(){

    _initialized=false;
    return(false); // We don't want this object automatically destroyed at the end of simulation
}

PLUGINLIB_EXPORT_CLASS(Force_sensorHandler, GenericObjectHandler)
