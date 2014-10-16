
#include <pluginlib/class_list_macros.h>

#include <rigid_body_handler/GetTwistHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>


#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>


GetTwistHandler::GetTwistHandler() : GenericObjectHandler(),
    _ObjTwistFrequency(-1),
    _lastPublishedObjTwistTime(0.0),
    _isStatic(true){
}

GetTwistHandler::~GetTwistHandler(){
}

unsigned int GetTwistHandler::getObjectType() const {
    return CustomDataHeaders::OBJ_TWIST_DATA_MAIN;
}

void GetTwistHandler::synchronize(){
    // We update GetTwistHandler's data from its associated scene object custom data:
    // 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
    int buffSize = simGetObjectCustomDataLength(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER);
    char* datBuff=new char[buffSize];
    simGetObjectCustomData(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
    delete[] datBuff;
    // 2. From that retrieved data, try to extract sub-data with the OBJ_TWIST_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;

    if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::OBJ_TWIST_DATA_MAIN,tempMainData)){ // Yes, the tag is there. For now we only have to synchronize _maxVelocity:

       	// Remove # chars for compatibility with ROS
		_associatedObjectName = simGetObjectName(_associatedObjectID);
		std::string objectName(_associatedObjectName);
		std::replace( objectName.begin(), objectName.end(), '#', '_');
		_pub = _nh.advertise<geometry_msgs::TwistStamped>(objectName+"/twist", 1);
		std::stringstream streamtemp;

    	int temp_freq = CAccess::pop_int(tempMainData);
    	if (temp_freq > 0){
    		_ObjTwistFrequency = temp_freq;
    		//std::cout << "Found Twist target in " << _associatedObjectName << ". Frequency publisher: " << _ObjTwistFrequency << std::endl;

    		streamtemp << "- [Twist Target] in '" << _associatedObjectName << "'. Frequency publisher: " << _ObjTwistFrequency << "." << std::endl;
    		ConsoleHandler::printInConsole(streamtemp);
      	} else {
      		//std::cout << "Found Twist target in " << _associatedObjectName << " at the simulation frequency. " << std::endl;

    		streamtemp << "- [Twist Target] in '" << _associatedObjectName << "'. Frequency publisher: Simulation frequency. " << std::endl;
    	    ConsoleHandler::printInConsole(streamtemp);
      	}
    }


}

void GetTwistHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    ros::Time now = ros::Time::now();


    const simFloat currentSimulationTime = simGetSimulationTime();

    if ((currentSimulationTime-_lastPublishedObjTwistTime) >= 1.0/_ObjTwistFrequency){

    	Eigen::Quaternion< simFloat > orientation; //(x,y,z,w)
    	Eigen::Matrix<simFloat, 3, 1> linVelocity;
    	Eigen::Matrix<simFloat, 3, 1> angVelocity;
    	bool error = false;

    	// Get object velocity. If the object is not static simGetVelocity is more accurate.
    	if (_isStatic){
    		error = error || simGetObjectVelocity(_associatedObjectID, linVelocity.data(), angVelocity.data()) == -1;
    	} else {
    		error = error || simGetVelocity(_associatedObjectID, linVelocity.data(), angVelocity.data()) == -1;
    	}

    	// Get object orientation
    	error = error || simGetObjectQuaternion(_associatedObjectID, -1, orientation.coeffs().data()) == -1;

    	if(!error){

    		linVelocity = orientation.conjugate()*linVelocity; // Express linear velocity in body frame
			angVelocity = orientation.conjugate()*angVelocity; // Express angular velocity in body frame

    		// Fill the status msg
			geometry_msgs::TwistStamped msg;

			msg.twist.linear.x = linVelocity[0];
			msg.twist.linear.y = linVelocity[1];
			msg.twist.linear.z = linVelocity[2];
			msg.twist.angular.x = angVelocity[0];
			msg.twist.angular.y = angVelocity[1];
			msg.twist.angular.z = angVelocity[2];

			msg.header.stamp = now;
			_pub.publish(msg);
			_lastPublishedObjTwistTime = currentSimulationTime;

    	} else {
    	    std::stringstream ss;
    	    ss << "- [" << _associatedObjectName << "] Error getting object velocity and/or orientation." << std::endl;;
    	    ConsoleHandler::printInConsole(ss);
    	}

    }

}


void GetTwistHandler::_initialize(){
    if (_initialized)
        return;


    std::vector<int> toExplore;
    toExplore.push_back(_associatedObjectID); // We start exploration with the base of the quadrotor-tree
    while (toExplore.size()!=0)
    {
        int objHandle=toExplore[toExplore.size()-1];
        toExplore.pop_back();
        // 1. Add this object's children to the list to explore:
        int index=0;
        int childHandle=simGetObjectChild(objHandle,index++);
        while (childHandle!=-1) {
            toExplore.push_back(childHandle);
//            std::cout << "Adding " << simGetObjectName(childHandle) << " to exploration list." << std::endl;
            childHandle=simGetObjectChild(objHandle,index++);
        }
        // 2. Now check if this object has one of the tags we are looking for:
        // a. Get all the developer data attached to this scene object (this is custom data added by the developer):
        int buffSize=simGetObjectCustomDataLength(objHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER);
        if (buffSize!=0) { // Yes there is some custom data written by us (the developer with the DEVELOPER_DATA_HEADER header)
            char* datBuff=new char[buffSize];
            simGetObjectCustomData(objHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
            std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
            delete[] datBuff;
            // b. From that retrieved data, try to extract sub-data with the searched tags:
            std::vector<unsigned char> quadrotorTagData;


        }
    }
    _lastPublishedObjTwistTime = -1e5;

    //Check if the object is static
    simGetObjectIntParameter(_associatedObjectID, 3003, &_isStatic);
    if(_isStatic){
		std::stringstream ss;
		ss << "- [" << _associatedObjectName << "] WARNING: getting velocity of a static object might give inaccurate results." << std::endl;;
		ConsoleHandler::printInConsole(ss);
    }

    _initialized=true;

}


PLUGINLIB_EXPORT_CLASS(GetTwistHandler, GenericObjectHandler)
