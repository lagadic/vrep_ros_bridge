
#include <pluginlib/class_list_macros.h>

#include <rigid_body_handler/GetPoseHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>


GetPoseHandler::GetPoseHandler() : GenericObjectHandler(),
    _ObjPoseFrequency(-1),
    _lastPublishedObjPoseTime(0.0){
}

GetPoseHandler::~GetPoseHandler(){
}

unsigned int GetPoseHandler::getObjectType() const {
    return CustomDataHeaders::OBJ_POSE_DATA_MAIN;
}

void GetPoseHandler::synchronize(){
    // We update GetPoseHandler's data from its associated scene object custom data:
    // 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
    int buffSize = simGetObjectCustomDataLength(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER);
    char* datBuff=new char[buffSize];
    simGetObjectCustomData(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
    delete[] datBuff;
    // 2. From that retrieved data, try to extract sub-data with the OBJ_POSE_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;


    if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::OBJ_POSE_DATA_MAIN,tempMainData)){ // Yes, the tag is there. For now we only have to synchronize _maxVelocity:

      	// Remove # chars for compatibility with ROS
		_associatedObjectName = simGetObjectName(_associatedObjectID);
		std::string objectName(_associatedObjectName);
		std::replace( objectName.begin(), objectName.end(), '#', '_');
		_pub = _nh.advertise<geometry_msgs::PoseStamped>(objectName+"/pose", 1);
		std::stringstream streamtemp;


    	int temp_freq = CAccess::pop_int(tempMainData);

    	if (temp_freq > 0){
    		_ObjPoseFrequency = temp_freq;
    		//std::cout << "Frequency Pose Target = " << _ObjPoseFrequency << std::endl;
    		streamtemp << "- [Pose Target] in '" << _associatedObjectName << "'. Frequency publisher: " << _ObjPoseFrequency << "." << std::endl;
    		ConsoleHandler::printInConsole(streamtemp);
    	} else {
      		//std::cout << "Found Twist target in " << _associatedObjectName << " at the simulation frequency. " << std::endl;

    		streamtemp << "- [Pose Target] in '" << _associatedObjectName << "'. Frequency publisher: Simulation frequency. " << std::endl;
    	    ConsoleHandler::printInConsole(streamtemp);
    	}

    }



}

void GetPoseHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    ros::Time now = ros::Time::now();

    const simFloat currentSimulationTime = simGetSimulationTime();

    if ((currentSimulationTime-_lastPublishedObjPoseTime) >= 1.0/_ObjPoseFrequency){

    	Eigen::Matrix<simFloat, 3, 1> position;
    	Eigen::Quaternion< simFloat > orientation; //(x,y,z,w)

    	// Get Position and Orientation of the object
    	if(simGetObjectPosition(_associatedObjectID, -1, position.data())!=-1 && simGetObjectQuaternion(_associatedObjectID, -1, orientation.coeffs().data())!=-1){

//    		// use only positive w (not necessary)
    		if (orientation.w()<0){
    		     orientation.coeffs() *=-1;
    		  }

    		// Fill the status msg
			geometry_msgs::PoseStamped msg;

			msg.pose.position.x = position[0];
			msg.pose.position.y = position[1];
			msg.pose.position.z = position[2];
			msg.pose.orientation.w = orientation.w();
			msg.pose.orientation.x = orientation.x();
			msg.pose.orientation.y = orientation.y();
			msg.pose.orientation.z = orientation.z();

			msg.header.stamp = now;
			_pub.publish(msg);
			_lastPublishedObjPoseTime = currentSimulationTime;

    	}

    }

}


void GetPoseHandler::_initialize(){
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
   _lastPublishedObjPoseTime = -1e5;
    _initialized=true;
}

PLUGINLIB_EXPORT_CLASS(GetPoseHandler, GenericObjectHandler)
