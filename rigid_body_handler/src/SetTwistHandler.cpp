
#include <pluginlib/class_list_macros.h>

#include <rigid_body_handler/SetTwistHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>


#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>


SetTwistHandler::SetTwistHandler() : GenericObjectHandler(),
_isStatic(false){
    // Set Object twist defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_set_obj_twist_data_main", (boost::lexical_cast<std::string>(int(SET_OBJ_TWIST_DATA_MAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_set_obj_twist_data_lin_gain", (boost::lexical_cast<std::string>(int(SET_OBJ_TWIST_DATA_LIN_GAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_set_obj_twist_data_ang_gain", (boost::lexical_cast<std::string>(int(SET_OBJ_TWIST_DATA_ANG_GAIN))).c_str());
}

SetTwistHandler::~SetTwistHandler(){
}

unsigned int SetTwistHandler::getObjectType() const {
	return SET_OBJ_TWIST_DATA_MAIN;
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

	if (CAccess::extractSerializationData(developerCustomData,SET_OBJ_TWIST_DATA_MAIN,tempMainData)){ // Yes, the tag is there. For now we only have to synchronize _maxVelocity:

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

	std::stringstream ss;

	Eigen::Quaternion < simFloat > orientation; //(x,y,z,w)
	Eigen::Matrix< simFloat, 3, 1> desLinVel((simFloat)_twistCommands.twist.linear.x,
			(simFloat)_twistCommands.twist.linear.y,
			(simFloat)_twistCommands.twist.linear.z);
	Eigen::Matrix< simFloat, 3, 1> desAngVel((simFloat)_twistCommands.twist.angular.x,
			(simFloat)_twistCommands.twist.angular.y,
			(simFloat)_twistCommands.twist.angular.z);

	// Input velocity is expected to be in body frame but V-Rep expects it to be in world frame.
	if(simGetObjectQuaternion(_associatedObjectID, -1, orientation.coeffs().data())!=-1){
	} else {
		ss << "- [" << _associatedObjectName << "] Error getting orientation. " << std::endl;
	}

	//	simResetDynamicObject(_associatedObjectID);

	//Set object velocity
	if (_isStatic){
		Eigen::Matrix<simFloat, 3, 1> position;
		simGetObjectPosition(_associatedObjectID, -1, position.data());
		const simFloat timeStep = simGetSimulationTimeStep();
		const simFloat angle = timeStep*desAngVel.norm();
		const Eigen::Matrix<simFloat, 3, 1> axis =
				(angle > 1e-6) ? desAngVel.normalized() : (Eigen::Matrix<simFloat, 3, 1>() << 1,0,0).finished();

		position += timeStep*(orientation*desLinVel);
		orientation *= Eigen::Quaternion<simFloat>(Eigen::AngleAxis<simFloat>(angle, axis));

		simSetObjectPosition(_associatedObjectID, -1, position.data());
		simSetObjectQuaternion(_associatedObjectID, -1, orientation.coeffs().data());
	} else {
		// Apply the linear velocity to the object
		//		simResetDynamicObject(_associatedObjectID);
		//		if(simSetObjectFloatParameter(_associatedObjectID, sim_shapefloatparam_init_velocity_x, desLinVel[0]) != 1 ||
		//				simSetObjectFloatParameter(_associatedObjectID, sim_shapefloatparam_init_velocity_y, desLinVel[1]) != 1 ||
		//				simSetObjectFloatParameter(_associatedObjectID, sim_shapefloatparam_init_velocity_z, desLinVel[2]) != 1 ||
		//				simSetObjectFloatParameter(_associatedObjectID, sim_shapefloatparam_init_velocity_a, desAngVel[0]) != 1 ||
		//				simSetObjectFloatParameter(_associatedObjectID, sim_shapefloatparam_init_velocity_b, desAngVel[1]) != 1 ||
		//				simSetObjectFloatParameter(_associatedObjectID, sim_shapefloatparam_init_velocity_g, desAngVel[2]) != 1) {
		//
		//			ss << "- [" << _associatedObjectName << "] Error setting object velocity." << std::endl;
		//		} else {
		//			ss << "- [" << _associatedObjectName << "] Object velocity correctly set to ["
		//					<< desLinVel.transpose() << " " << desAngVel.transpose() << "]." << std::endl;
		//		}

		simFloat mass;
		Eigen::Matrix<simFloat, 3, 3> inertiaMatrix;
		Eigen::Matrix<simFloat, 3, 1> centerOfMass, linVel, angVel, gravity;
		_simGetGravity(gravity.data());

		simFloat matrix[12];
		simGetObjectMatrix(_associatedObjectID, -1, matrix);
		simGetShapeMassAndInertia(_associatedObjectID, &mass, inertiaMatrix.data(), centerOfMass.data(), matrix);

		simGetVelocity(_associatedObjectID, linVel.data(), angVel.data());
		angVel = orientation.conjugate()*angVel;

		Eigen::Matrix<simFloat, 3, 1> force = -mass*(gravity + _linGain*(linVel-orientation*desLinVel));
		Eigen::Matrix<simFloat, 3, 1> torque = orientation*(angVel.cross(inertiaMatrix*angVel) - inertiaMatrix*_angGain*(angVel-desAngVel));

		simAddForceAndTorque(_associatedObjectID, force.data(), torque.data());
	}

	if (ss.rdbuf()->in_avail()){
		ConsoleHandler::printInConsole(ss);
	}

}


void SetTwistHandler::_initialize(){
	if (_initialized)
		return;

	std::stringstream ss;

	std::string objectName(_associatedObjectName);
	std::replace( objectName.begin(), objectName.end(), '#', '_');
	_sub = _nh.subscribe(objectName+"/SetTwist", 1, &SetTwistHandler::TwistCommandCallback, this);

	//Check if the object is static
	simGetObjectIntParameter(_associatedObjectID, sim_shapeintparam_static, &_isStatic);
	if(_isStatic){
		std::stringstream ss;
		ss << "- [" << _associatedObjectName << "] WARNING: setting velocity of a static object might give inaccurate results." << std::endl;;
		ConsoleHandler::printInConsole(ss);
	}

	std::vector<unsigned char> developerCustomData;
	getDeveloperCustomData(developerCustomData);
	std::vector<unsigned char> tempMainData;

	if (CAccess::extractSerializationData(developerCustomData,SET_OBJ_TWIST_DATA_LIN_GAIN,tempMainData)){
		_linGain=CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Linear gain set to " << _linGain << "." << std::endl;
	} else {
		_linGain = 1.0;
		ss << "- [" << _associatedObjectName << "] Linear gain not specified. Using " << _linGain << " as default." << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,SET_OBJ_TWIST_DATA_ANG_GAIN,tempMainData)){
		_angGain=CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Angular gain set to " << _angGain << "." << std::endl;
	} else {
		_angGain = 1.0;
		ss << "- [" << _associatedObjectName << "] Angular gain not specified. Using " << _angGain << " as default." << std::endl;
	}

	ConsoleHandler::printInConsole(ss);
	_initialized=true;
}



void SetTwistHandler::TwistCommandCallback(const geometry_msgs::TwistStamped& msg){

	_twistCommands = msg;

}

PLUGINLIB_EXPORT_CLASS(SetTwistHandler, GenericObjectHandler)


