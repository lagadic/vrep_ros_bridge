
#include <pluginlib/class_list_macros.h>

#include <rigid_body_handler/SetTwistHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>


#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>


SetTwistHandler::SetTwistHandler() : GenericObjectHandler(){
}

SetTwistHandler::~SetTwistHandler(){
}

unsigned int SetTwistHandler::getObjectType() const {
    return CustomDataHeaders::SET_OBJ_TWIST_DATA_MAIN;
}

void SetTwistHandler::synchronize(){
    // We update SetTwistHandler's data from its associated scene object custom data:
    // 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
    int buffSize = simGetObjectCustomDataLength(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER);
    char* datBuff=new char[buffSize];
    simGetObjectCustomData(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
    delete[] datBuff;
    // 2. From that retrieved data, try to extract sub-data with the SET_OBJ_TWIST_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;

    if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::SET_OBJ_TWIST_DATA_MAIN,tempMainData)){ // Yes, the tag is there. For now we only have to synchronize _maxVelocity:

       	// Remove # chars for compatibility with ROS
		_associatedObjectName = simGetObjectName(_associatedObjectID);
		std::string objectName(_associatedObjectName);
		std::replace( objectName.begin(), objectName.end(), '#', '_');
		//_pub = _nh.advertise<geometry_msgs::TwistStamped>(objectName+"/twist", 1);
		std::stringstream streamtemp;


		streamtemp << "- [Set Twist Target] in '" << _associatedObjectName << "'. Subscribed on the topic "<< objectName<<"/SetTwist"<< std::endl;
		ConsoleHandler::printInConsole(streamtemp);

    }



}

void SetTwistHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    Eigen::Quaternion < simFloat > orientation; //(x,y,z,w)
    Eigen::Matrix< simFloat, 3, 1> Velocity((simFloat)_twistCommands.twist.linear.x ,(simFloat)_twistCommands.twist.linear.y, (simFloat)_twistCommands.twist.linear.z);

   if( simGetObjectQuaternion(_associatedObjectID, -1, orientation.coeffs().data())!=-1)

   {


	   Velocity = orientation * Velocity ;

   }



    simResetDynamicObject(_associatedObjectID);

/*    // Apply the linear velocity to the object
    if(simSetObjectFloatParameter( _associatedObjectID,3000, _twistCommands.twist.linear.x)
    	&& simSetObjectFloatParameter( _associatedObjectID,3001, _twistCommands.twist.linear.y)
    	&& simSetObjectFloatParameter( _associatedObjectID,3002, _twistCommands.twist.linear.z)==-1) {
                   std::stringstream ss;
                   ss << "- [" << _associatedObjectName << "] Error setting linear velocity. ";
                   ConsoleHandler::printInConsole(ss);
               }
    // Apply the angular velocity to the object
    if(simSetObjectFloatParameter( _associatedObjectID,3020, _twistCommands.twist.angular.x)
    	&& simSetObjectFloatParameter( _associatedObjectID,3021, _twistCommands.twist.angular.y)
    	&& simSetObjectFloatParameter( _associatedObjectID,3022, _twistCommands.twist.angular.z)==-1) {
                   std::stringstream ss;
                   ss << "- [" << _associatedObjectName << "] Error setting angular velocity. ";
                   ConsoleHandler::printInConsole(ss);
               }*/




    // Apply the linear velocity to the object
    if(simSetObjectFloatParameter( _associatedObjectID,3000, Velocity[0])
    	&& simSetObjectFloatParameter( _associatedObjectID,3001, Velocity[1])
    	&& simSetObjectFloatParameter( _associatedObjectID,3002, Velocity[2])==-1) {
                   std::stringstream ss;
                   ss << "- [" << _associatedObjectName << "] Error setting linear velocity. ";
                   ConsoleHandler::printInConsole(ss);
               }
    // Apply the angular velocity to the object
    if(simSetObjectFloatParameter( _associatedObjectID,3020, _twistCommands.twist.angular.x)
    	&& simSetObjectFloatParameter( _associatedObjectID,3021, _twistCommands.twist.angular.y)
    	&& simSetObjectFloatParameter( _associatedObjectID,3022, _twistCommands.twist.angular.z)==-1) {
                   std::stringstream ss;
                   ss << "- [" << _associatedObjectName << "] Error setting angular velocity. ";
                   ConsoleHandler::printInConsole(ss);
               }







}


void SetTwistHandler::_initialize(){
    if (_initialized)
        return;


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
//        int buffSize=simGetObjectCustomDataLength(objHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER);
//        if (buffSize!=0) { // Yes there is some custom data written by us (the developer with the DEVELOPER_DATA_HEADER header)
//            char* datBuff=new char[buffSize];
//            simGetObjectCustomData(objHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
//            std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
//            delete[] datBuff;
//            // b. From that retrieved data, try to extract sub-data with the searched tags:
//            std::vector<unsigned char> quadrotorTagData;
//
//
//        }
//    }

    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');
    _sub = _nh.subscribe(objectName+"/SetTwist", 1, &SetTwistHandler::TwistCommandCallback, this);

   //_lastPublishedObjTwistTime = -1e5;
    _initialized=true;
}



void SetTwistHandler::TwistCommandCallback(const geometry_msgs::TwistStamped& msg){

	_twistCommands = msg;

}


PLUGINLIB_EXPORT_CLASS(SetTwistHandler, GenericObjectHandler)

