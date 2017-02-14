
#include <pluginlib/class_list_macros.h>
#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <quadrotor_tk_handler/Quadrotor_tk_Handler.h>
#include <stdio.h>
#include <v_repLib.h>

#include <telekyb_msgs/TKState.h>
#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/Geometry>

#include <vrep_ros_plugin/access.h>
#include <vrep_ros_plugin/ConsoleHandler.h>

Quadrotor_tk_Handler::Quadrotor_tk_Handler() : GenericObjectHandler(),
_torqueToForceRatio(0.01738),
_handleOfCoM(-1),
_ctrlMode(CustomDataHeaders::DIRECT),
_tkMotorCommands(4,0),
//_tkCommands(3,0),
_quadrotorMass(1.0),
// Before changing the following 3 parameters be careful and look closely into the controller which is implemented below
_att_cutoff(5.0),
_att_damping(0.42),
_kp_yaw(1),
_lastReceivedCmdTime(ros::Time::now()),
_previousTime(ros::Time::now()),
_integralTermRoll(0.0),
_integralTermPitch(0.0)
{
}

Quadrotor_tk_Handler::~Quadrotor_tk_Handler(){
}

unsigned int Quadrotor_tk_Handler::getObjectType() const {
	return CustomDataHeaders::QUADROTOR_TK_DATA_MAIN;
}

/*void Quadrotor_tk_Handler::setID(int newID){
    _id=newID;
}

int Quadrotor_tk_Handler::getID(){
    return(_id);
}*/

/*
void Quadrotor_tk_Handler::setAssociatedObject(int objID,int objUniqueID){
    _associatedObjectID=objID;
    _associatedObjectUniqueID=objUniqueID;
}

int Quadrotor_tk_Handler::getAssociatedObject(){
    return(_associatedObjectID);
}

int Quadrotor_tk_Handler::getAssociatedObjectUniqueId(){
    return(_associatedObjectUniqueID);
}

void Quadrotor_tk_Handler::synchronizeSceneObject(){
    // We update Quadrotor_tk_Handler's associated scene object custom data:
    putQuadrotorTagToSceneObject(_associatedObjectID,_torqueToForceRatio);
}
 */

void Quadrotor_tk_Handler::synchronize(){
	//    // We update Quadrotor_tk_Handler's data from its associated scene object custom data:
	//    // 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
	//    int buffSize=simGetObjectCustomDataLength(_associatedObjectID,CustomDataHeaders::DEVELOPER_DATA_HEADER);
	//    char* datBuff=new char[buffSize];
	//    simGetObjectCustomData(_associatedObjectID,CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
	//    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
	//    delete[] datBuff;
	//    // 2. From that retrieved data, try to extract sub-data with the QUADROTOR_DATA_TF_RATIO tag:
	//    std::vector<unsigned char> tempMainData;
	//    if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::QUADROTOR_DATA_TF_RATIO,tempMainData))
	//    { // Yes, the tag is there. For now we only have to synchronize _maxVelocity:
	//        _torqueToForceRatio=CAccess::pop_float(tempMainData);
	//        std::cout << "torque to force ratio: " << _torqueToForceRatio << std::endl;
	//    }


	simFloat intertia[9];
	simFloat compos[3];

	// Get the mass of the quadcopter
	simGetShapeMassAndInertia(_associatedObjectID,&_quadrotorMass,intertia,compos,NULL);

	// Remove # chars for compatibility with ROS
	_associatedObjectName = simGetObjectName(_associatedObjectID);
	std::string objectName(_associatedObjectName);
	std::replace( objectName.begin(), objectName.end(), '#', '_');

	// Publisher of the quadropter status
	_pub = _nh.advertise<telekyb_msgs::TKState>(objectName+"/status", 1000);

	// Publisher of the IMU
	_pubIMU = _nh.advertise<sensor_msgs::Imu>(objectName+"/IMU", 1000);

	// Subscriber of the command from TeleKib
	//_sub = _nh.subscribe("/Telekyb/TelekybCore_0/Commands", 1000, &Quadrotor_tk_Handler::tkMotorCommandsCallback, this);
}


//void Quadrotor_tk_Handler::setThrustToForceRatio(float thrustToForceRatio){
//    _torqueToForceRatio=thrustToForceRatio;
//    synchronizeSceneObject(); // make sure the associated scene object has the same values stored as this
//}
//
//float Quadrotor_tk_Handler::getThrustToForceRatio(){
//    return(_torqueToForceRatio);
//}

//void Quadrotor_tk_Handler::putQuadrotorTagToSceneObject(int objectHandle,float torqueToForceRatio){
//    // This creates/updates a QUADROTOR_DATA_MAIN tag
//    // 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
//    int buffSize=simGetObjectCustomDataLength(objectHandle,DEVELOPER_DATA_HEADER);
//    char* datBuff=new char[buffSize];
//    simGetObjectCustomData(objectHandle,DEVELOPER_DATA_HEADER,datBuff);
//    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
//    delete[] datBuff;
//    // 2. From that retrieved data, extract sub-data with the QUADROTOR_DATA_MAIN tag, update it, and add it back to the retrieved data:
//    std::vector<unsigned char> tempMainData;
//    CAccess::extractSerializationData(developerCustomData,QUADROTOR_DATA_MAIN,tempMainData);
//    tempMainData.clear(); // we discard the old value (if present)
//    CAccess::push_float(tempMainData,torqueToForceRatio); // we replace it with the new value
//    CAccess::insertSerializationData(developerCustomData,QUADROTOR_DATA_MAIN,tempMainData);
//    // 3. We add/update the scene object with the updated custom data:
//    simAddObjectCustomData(objectHandle,DEVELOPER_DATA_HEADER,(simChar*)&developerCustomData[0],int(developerCustomData.size()));
//}

void Quadrotor_tk_Handler::handleSimulation(){
	// called when the main script calls: simHandleModule (it is called time by time)
	if(!_initialized){
		_initialize();
	}

	// TimeStep Computation
	ros::Time now = ros::Time::now();
	ros::Duration timeStep = now - _previousTime;
	//DEBUG
	//	std::cout<< "DEBUG: TimeStep: " << timeStep << std::endl;

	// Position of the quadrotor (x,y,z)
	Eigen::Matrix<simFloat, 3, 1> position;
	// Orientation of the quadrotor
	Eigen::Quaternion< simFloat > orientation; //(x,y,z,w)
	Eigen::Matrix<simFloat, 3, 1> linVelocity;
	Eigen::Matrix<simFloat, 3, 1> angVelocity;
	const static Eigen::Quaternion< simFloat > nwuToNed(0,1,0,0);

	if(simGetObjectPosition(_handleOfCoM, -1, position.data())!=-1 &&
			simGetObjectQuaternion(_handleOfCoM, -1, orientation.coeffs().data())!=-1 &&
			simGetObjectVelocity(_handleOfCoM, linVelocity.data(), angVelocity.data())!=-1){
		//DEBUG
		//std::cout<<"DEBUG: Rotation ";

		position = nwuToNed*position;
		linVelocity = nwuToNed*linVelocity;
		angVelocity = orientation.conjugate()*angVelocity; // Express angular velocity in body frame
		orientation = nwuToNed*orientation; // change world frame to north-east-down


		//angVelocity = nwuToNed*angVelocity;
		// use only positive w (not necessary)
		if (orientation.w()<0){
			orientation.coeffs() *=-1;
		}

		// Fill the status msg
		telekyb_msgs::TKState msg;
		msg.header.stamp = now;
		msg.pose.position.x = position[0];
		msg.pose.position.y = position[1];
		msg.pose.position.z = position[2];
		msg.pose.orientation.w = orientation.w();
		msg.pose.orientation.x = orientation.x();
		msg.pose.orientation.y = orientation.y();
		msg.pose.orientation.z = orientation.z();
		msg.twist.linear.x = linVelocity[0];
		msg.twist.linear.y = linVelocity[1];
		msg.twist.linear.z = linVelocity[2];
		msg.twist.angular.x = angVelocity[0];
		msg.twist.angular.y = angVelocity[1];
		msg.twist.angular.z = angVelocity[2];
		std::stringstream ss;
		//DEBUG:
		//std::cout << "Filling the TKState msg" << std::endl;
		_pub.publish(msg);
	}

	simFloat TotforceZ = 0.0;

	if (_ctrlMode == CustomDataHeaders::DIRECT){
		//DEBUG
		//std::cout<<"DEBUG: DIRECT MODE ";

		for(uint motorIdx = 0; motorIdx < 4; ++motorIdx){
			const simFloat force[3] = {0.0, 0.0, -(simFloat)_tkMotorCommands[motorIdx]};
			const simFloat torque[3] = {0.0, 0.0, _torqueToForceRatio*(1-2*((int)(motorIdx/2)))*(simFloat)_tkMotorCommands[motorIdx]};

			// Compute the net force
			TotforceZ =+ -(simFloat)_tkMotorCommands[motorIdx];

			// Try to apply a force to the object
			if(simAddForce(_associatedObjectID, _jointPosition[motorIdx], force)==-1){
				simSetLastError( _associatedObjectName.c_str(), "Error applying force.");
			} else if(simAddForceAndTorque(_associatedObjectID, NULL, torque)==-1){
				simSetLastError( _associatedObjectName.c_str(), "Error applying torque.");
			} else if(simSetJointTargetVelocity(_handleOfJoint[motorIdx], 10*_tkMotorCommands[motorIdx])==-1){
				simSetLastError( simGetObjectName(_handleOfJoint[motorIdx]), "Error applying velocity.");
			}
		}
	} else if (_ctrlMode == CustomDataHeaders::INTERNAL){

		//		timeStep=now-previousTime;
		//DEBUG
		//		std::cout<<"INTERNAL MODE "<< timeStep;
		//DEBUG
		//std::cout<<"INTERNAL MODE ";

		// Compute the net force
		TotforceZ = (simFloat)_tkCommands.thrust;

		//		Eigen::Matrix< simFloat, 3, 1> rpy = orientation.toRotationMatrix().eulerAngles(2,1,0);
		//		Eigen::Matrix< simFloat, 3, 3> rotMatrix = orientation.toRotationMatrix();
		//		Eigen::Matrix< simFloat, 3, 1> rpy(
		//				atan2(rotMatrix(3,1),rotMatrix(3,2)),
		//				acos(rotMatrix(3,3)),
		//				-atan2(rotMatrix(1,3),rotMatrix(2,3)));
		//
		//				atan2(rotMatrix(3,2),rotMatrix(3,3)),
		//				atan2(-rotMatrix(3,1),sqrt(rotMatrix(3,2)*rotMatrix(3,2)+rotMatrix(3,3)*rotMatrix(3,3))),
		//				atan2(rotMatrix(2,1),rotMatrix(1,1)));

		Eigen::Matrix< simFloat, 3, 3> R = orientation.toRotationMatrix();
		Eigen::Matrix< simFloat, 3, 1> rpy;
		rpy(0) = atan2((double)R(2,1), (double)R(2,2));
		rpy(1) = atan2(-R(2,0), sqrt( R(2,1)*R(2,1) + R(2,2)*R(2,2) ) );
		rpy(2) = atan2((double)R(1,0),(double)R(0,0));

		//		std::cout << "DEBUG: (_tkCommands.roll): " <<  (_tkCommands.roll) <<std::endl;
		//		std::cout << "DEBUG: rpy: " << rpy(0) << "  "<< rpy(1) << "  " << rpy(2) <<std::endl;
		//		std::cout << "DEBUG: R: " << R <<std::endl;
		//
		//		std::cout << "DEBUG: R(3,1): " << R(3,1) <<std::endl;
		//		std::cout << "DEBUG: R(3,2): " << R(3,2) <<std::endl;
		//		std::cout << "DEBUG: R(3,3): " << R(3,3) <<std::endl;
		// Definition of sin_r,cos_r etc.
		const double cos_r = cos((double)rpy(0));
		//std::cout << "DEBUG: cos_r: " <<  cos_r <<std::endl;
		const double sin_r = sin((double)rpy(0));
		//std::cout << "DEBUG: sin_r: " <<  sin_r <<std::endl;
		const double cos_p = cos((double)rpy(1));
		//std::cout << "DEBUG: cos_p: " <<  cos_p <<std::endl;
		const double sin_p = sin((double)rpy(1));
		//std::cout << "DEBUG: sin_p: " <<  sin_p <<std::endl;
		const double tan_p = tan((double)rpy(1));

		const Eigen::Matrix<simFloat, 3, 3> angVelToEulerRate = (Eigen::Matrix<simFloat, 3, 3>() <<
				1.0, sin_r*tan_p, cos_r*tan_p,
				0.0,       cos_r,      -sin_r,
				0.0, sin_r/cos_p, cos_r/cos_p).finished();

		Eigen::Matrix< simFloat, 3, 1> rpyRate = angVelToEulerRate*angVelocity;
		const simFloat kp_att = _att_cutoff;
		const simFloat kd_att = _att_damping;

		// Errors
		const simFloat errorRoll = (_tkCommands.roll)-(rpy(0));
		const simFloat errorPitch = (_tkCommands.pitch)-(rpy(1));

		// Debug: the 2 following instructions are needed to give to the quadrotor a commanded
		// 		  roll angle of 0 deg and a commanded pitch angle of -20 deg.
		//        To apply these angles you should also comment the line:
		// 		  Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*orientation*Eigen::Matrix< simFloat, 3, 1>(0.0,0.0,(simFloat)_tkCommands.thrust);
		//        and substitute it with the following line:
		// 	      Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*Eigen::Matrix< simFloat, 3, 1>(0.0,0.0,-9.81);
		// 		  which is needed to keep the quadrotor at the desired angles.
	/*	const simFloat errorRoll = (-40*3.14/180)-(rpy(0));
		const simFloat errorPitch = (-40*3.14/180)-(rpy(1));*/


		// Computaton of the integral terms
		_integralTermRoll = _integralTermRoll + timeStep.toSec() * errorRoll;
		_integralTermPitch = _integralTermPitch + timeStep.toSec() * errorPitch;
		if(_integralTermRoll<0.0001)_integralTermRoll=0.0;
		if(_integralTermPitch<0.0001)_integralTermPitch=0.0;


		Eigen::Matrix< simFloat, 3, 1> rpyTorque(
				//roll
				kp_att*(errorRoll) + kd_att*(0-rpyRate(0))+0*0.001*_integralTermRoll,
				//pitch
				//				0.1*kp_att*(errorPitch) + 0.02*kd_att*(0-rpyRate(1))+0*0.002*_integralTermPitch,
				kp_att*(errorPitch) + kd_att*(0-rpyRate(1))+0*0.002*_integralTermPitch,
				//yaw
				_kp_yaw*(_tkCommands.yaw- rpyRate(2)));  //sign is the opposite of the yaw angular velocity

		//DEBUG
		//		std::cout << "DEBUG: (_tkCommands.roll): " <<  (_tkCommands.roll) <<std::endl;
		//		std::cout << "DEBUG: rpy: " << rpy(0) << "  "<< rpy(1) << "  " << rpy(2) <<std::endl;
		//		std::cout << "DEBUG: sin(_tkCommands.pitch): " <<  sin(_tkCommands.pitch) <<std::endl;
		//		std::cout << "DEBUG: sin(rpy(1)): " << sin(rpy(1)) <<std::endl;
		//		std::cout << "DEBUG: errorRoll: " << errorRoll <<std::endl;
		//		std::cout << "DEBUG: errorPitch: " << errorPitch <<std::endl;
//		std::cout <<  _associatedObjectName.c_str() << " " << "_tkCommands.yaw: " << _tkCommands.yaw <<std::endl;
//		std::cout <<  _associatedObjectName.c_str() << " " << "rpyRate(2): " << rpyRate(2) <<std::endl;

		rpyTorque = nwuToNed*orientation*rpyTorque; //rotate torque to world frame
		//const Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*Eigen::Matrix< simFloat, 3, 1>(0.0,0.0,(simFloat)_tkCommands.thrust);

		Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*orientation*Eigen::Matrix< simFloat, 3, 1>(0.0,0.0,(simFloat)_tkCommands.thrust);

//		Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*Eigen::Matrix< simFloat, 3, 1>(0.0,0.0,-9.81);
		// To Delete Probably
		//		if(worldForce(0)<0.001 && worldForce(0)>-0.001)worldForce(0)=0;
		//		if(worldForce(1)<0.001 && worldForce(1)>-0.001)worldForce(1)=0;


		//DEBUG torques and forces
		//		std::cout << "DEBUG: rpyTorque: " << rpyTorque(0) <<"|"<< rpyTorque(1) <<"|"<< rpyTorque(2) <<"|" <<std::endl;
		//		std::cout << "DEBUG: forces: " << worldForce(0) <<"|"<< worldForce(1) <<"|"<< worldForce(2) <<"|" <<std::endl;

		//        std::stringstream ss;
		//        ss << "applying force : [" << worldForce.transpose() << std::endl;
		//        ConsoleHandler::printInConsole(ss);
		if(simAddForceAndTorque(_associatedObjectID, worldForce.data(), rpyTorque.data())==-1){
			//							if(simAddForceAndTorque(_associatedObjectID, worldForce.data(), 0)==-1){
			simSetLastError( _associatedObjectName.c_str(), "Error applying force.");
		} else {
			for (uint motorIdx = 0; motorIdx < 4; ++motorIdx){
				// the following is needed to rotate the propellers
				if(simSetJointTargetVelocity(_handleOfJoint[motorIdx], _tkCommands.thrust)==-1){
					simSetLastError( simGetObjectName(_handleOfJoint[motorIdx]), "Error applying velocity.");
				}
			}
		}
		_previousTime=now;
	}

	Eigen::Matrix<simFloat, 3, 1> TotCommandforces(0.0, 0.0, TotforceZ);
	Eigen::Matrix<simFloat, 3, 1> VectorG (0.0, 0.0, 9.8);
	VectorG = orientation.conjugate()*VectorG; // Express angular velocity in body frame

	// Fill the IMU msg
	sensor_msgs::Imu msg1;
	msg1.header.stamp = ros::Time::now();
	msg1.angular_velocity.x = angVelocity[0];
	msg1.angular_velocity.y = angVelocity[1];
	msg1.angular_velocity.z = angVelocity[2];
	msg1.linear_acceleration.x = TotCommandforces[0]/_quadrotorMass + VectorG[0];
	msg1.linear_acceleration.y = TotCommandforces[1]/_quadrotorMass + VectorG[1];
	msg1.linear_acceleration.z = TotCommandforces[2]/_quadrotorMass + VectorG[2];

	Eigen::Vector3d acc(msg1.linear_acceleration.x, msg1.linear_acceleration.y, msg1.linear_acceleration.z);
	Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(acc, Eigen::Vector3d(0,0,1));
	msg1.orientation.w = q.w();
	msg1.orientation.x = q.x();
	msg1.orientation.y = q.y();
	msg1.orientation.z = q.z();

	_pubIMU.publish(msg1); // Publish the Imu message in ROS

	if ((now-_lastReceivedCmdTime).toSec() > 0.1){
		_tkMotorCommands = std::vector<double>(4,0);
		_tkCommands.pitch = 0.0;
		_tkCommands.roll = 0.0;
		_tkCommands.yaw = 0.0;
		_tkCommands.thrust = 0.0;//_quadrotorMass*9.8;
		// Debug Print
		//		std::stringstream ss;
		//		ss << "I am here" << std::endl;
		//		simAddStatusbarMessage(ss.str().c_str());
		//		ConsoleHandler::printInConsole(ss);
		if ((now-_lastPrintedMsg).toSec() >= 1){
			std::stringstream ss;
			ss << "- [" << _associatedObjectName << "] No command received since more than " << (now-_lastReceivedCmdTime).toSec() << "s!" << std::endl;
			simAddStatusbarMessage(ss.str().c_str());
			ConsoleHandler::printInConsole(ss);
			_lastPrintedMsg = now;
		}
	}

	//    if ((now-_lastPrintedMsg).toSec() >= 1){
	//        std::stringstream sstream("I am alive at simulation time = ");
	//        sstream << simGetSimulationTime() << std::endl;
	//        simAddStatusbarMessage(sstream.str().c_str());
	//        _lastPrintedMsg = now;
	//    }
}


void Quadrotor_tk_Handler::_initialize(){
	if (_initialized)
		return;

	// get some data from the main object
	std::vector<unsigned char> developerCustomData;
	getDeveloperCustomData(developerCustomData);

	// 2. From that retrieved data, try to extract sub-data with the IMU_DATA_MAIN tag:
	std::vector<unsigned char> tempMainData;
	std::stringstream ss;

	if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::IMU_DATA_MASS,tempMainData)){
		//	if (false){
		_quadrotorMass=CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Setting mass to: " << _quadrotorMass << "." << std::endl;
	} else {
		_quadrotorMass = 24.0;
		ss << "- [" << _associatedObjectName << "] Mass of the quadrotor not specified. using " << _quadrotorMass << " as default."  << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::QUADROTOR_DATA_TF_RATIO,tempMainData)){
		_torqueToForceRatio=CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Setting torque to force ratio to: " << _torqueToForceRatio << "." << std::endl;
	} else {
		_torqueToForceRatio = 0.01738;
		ss << "- [" << _associatedObjectName << "] Torque to force ratio not specified. using " << _torqueToForceRatio << " as default."  << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::QUADROTOR_DATA_CTRL_MODE,tempMainData)){
		const int ctrlMode = CAccess::pop_int(tempMainData);
		if (ctrlMode == (int)(CustomDataHeaders::DIRECT)){
			_ctrlMode = CustomDataHeaders::DIRECT;
			ss << "- [" << _associatedObjectName << "] Using DIRECT control mode." << std::endl;
		} else if (ctrlMode == (int)(CustomDataHeaders::INTERNAL)){
			_ctrlMode = CustomDataHeaders::INTERNAL;
			ss << "- [" << _associatedObjectName << "] Using INTERNAL control mode." << std::endl;
		} else {
			_ctrlMode = CustomDataHeaders::DIRECT;
			ss << "- [" << _associatedObjectName << "] Invalid control mode specified. using DIRECT as default." << std::endl;
		}

	} else {
		_ctrlMode = CustomDataHeaders::DIRECT;
		ss << "- [" << _associatedObjectName << "] Control mode not specified. using DIRECT as default."  << std::endl;
	}

	// We need to find the handle of quadrotor motors, and CoM.
	for(uint motorIdx = 0; motorIdx < 4; ++motorIdx){
		_handleOfJoint[motorIdx] = -1;
	}
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
			for (uint motorIdx = 0; motorIdx < 4; ++motorIdx){
				if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::QUADROTOR_DATA_MOTOR_0+motorIdx,quadrotorTagData)){
					_handleOfJoint[motorIdx]=objHandle; // We found the QUADROTOR_DATA_MOTOR_i tag. This is the i-th motor!
					simGetObjectPosition(objHandle,_associatedObjectID,_jointPosition[motorIdx]);
					ss << "- [" << _associatedObjectName << "] Found motor " << motorIdx << " in position ["
							<< _jointPosition[motorIdx][0] << ", "
							<< _jointPosition[motorIdx][1] << ", "
							<< _jointPosition[motorIdx][2] << "]." <<  std::endl;
				}
			}
			if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::QUADROTOR_DATA_COM,quadrotorTagData)){
				_handleOfCoM=objHandle; // We found the QUADROTOR_DATA_MOTOR_i tag. This is the i-th motor!
				ss << "- [" << _associatedObjectName << "] Found CENTER OF MASS in '"<< simGetObjectName(objHandle) <<"'." << std::endl;
			}
		}
	}


	// Subscriber of the command from TeleKib
	std::string objectName(_associatedObjectName);
	std::replace( objectName.begin(), objectName.end(), '#', '_');

	if (_ctrlMode == CustomDataHeaders::DIRECT){
		try{
			_sub = _nh.subscribe(objectName+"/Commands", 1000, &Quadrotor_tk_Handler::tkMotorCommandsCallback, this);
		} catch (ros::Exception &e) {
			std::stringstream ss(e.what());
			ss << std::endl;
			ConsoleHandler::printInConsole(ss);
		}

	} else if (_ctrlMode == CustomDataHeaders::INTERNAL){
		try{
			std::string objectIdExtracted = objectName.substr(objectName.find('_')+1,objectName.size());

			std::cout<< std::endl << "- [" << objectName <<"] Subscribed for Commands (INTERNAL mode) on the ROS Topic: ";
			std::string rosTopicCommands = "/TeleKyb/TeleKybCore_" + objectIdExtracted + "/Commands";
			std::cout << rosTopicCommands << std::endl;

			// Subscribe to the Right topic according to the objectName.
			_sub = _nh.subscribe(rosTopicCommands, 1000, &Quadrotor_tk_Handler::tkCommandsCallback, this);

			// Example of right subscription for the object with objectName 'quadrotor_0'
			//_sub = _nh.subscribe("/TeleKyb/TeleKybCore_0/Commands", 1000, &Quadrotor_tk_Handler::tkCommandsCallback(), this)

		} catch (ros::Exception &e) {
			std::stringstream ss(e.what());
			ss << std::endl;
			ConsoleHandler::printInConsole(ss);
		}
	} else {
		simSetLastError( _associatedObjectName.c_str(), "Unknown control mode.");
	}



	ConsoleHandler::printInConsole(ss);
	_initialized=true;
}

void Quadrotor_tk_Handler::tkMotorCommandsCallback(const telekyb_msgs::TKMotorCommands::ConstPtr& msg){
	if (msg->force.size() == 4){
		_tkMotorCommands = msg->force;
		_lastReceivedCmdTime = ros::Time::now();
	} else {
		simSetLastError( _associatedObjectName.c_str(), "Received wrong command size.");
		//        std::cout << "Received wrong command containing " <<  msg->force.size() << " elements" << std::endl;
	}
}

void Quadrotor_tk_Handler::tkCommandsCallback(const telekyb_msgs::TKCommands::ConstPtr& msg){
	_tkCommands = *msg;
	_lastReceivedCmdTime = ros::Time::now();
}


PLUGINLIB_EXPORT_CLASS(Quadrotor_tk_Handler, GenericObjectHandler)




