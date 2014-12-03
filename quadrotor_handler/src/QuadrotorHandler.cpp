
#include <pluginlib/class_list_macros.h>
#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <quadrotor_handler/QuadrotorHandler.h>

#include <v_repLib.h>

//#include <telekyb_msgs/TKState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Geometry>

#include <vrep_ros_plugin/access.h>
#include <vrep_ros_plugin/ConsoleHandler.h>

QuadrotorHandler::QuadrotorHandler() : GenericObjectHandler(),
    _torqueToForceRatio(0.0),
    _handleOfCoM(-1),
    _quadrotorMass(0.8),
    _lastReceivedCmdTime(ros::Time::now()),
    _ObjStatusFrequency(-1),
    _lastPublishedStatusTime(0.0),
    _Commands(4,0)

    {


}

QuadrotorHandler::~QuadrotorHandler(){
}

unsigned int QuadrotorHandler::getObjectType() const {
    return CustomDataHeaders::QUADROTOR_DATA_MAIN;
}

void QuadrotorHandler::synchronize(){


    simFloat intertia[9];
    simFloat compos[3];

    // Get the mass of the quadropter
    simGetShapeMassAndInertia(_associatedObjectID,&_quadrotorMass,intertia,compos,NULL);

    // Remove # chars for compatibility with ROS
    _associatedObjectName = simGetObjectName(_associatedObjectID);
    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');

    // Publisher of the quadropter status
    //_pub = _nh.advertise<telekyb_msgs::TKState>(objectName+"/status", 1000);

    // Publisher of the quadrotor pose
	_pubPose = _nh.advertise<geometry_msgs::PoseStamped>(objectName+"/pose", 1);

	_pubTwist = _nh.advertise<geometry_msgs::TwistStamped>(objectName+"/twist", 1);

    // Publisher of the IMU
    //_pubIMU = _nh.advertise<sensor_msgs::Imu>("IMU/"+objectName, 1);


}



void QuadrotorHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }


   ros::Time now = ros::Time::now();

    Eigen::Matrix <simFloat, 3, 1> position;
    Eigen::Quaternion < simFloat > orientation; //(x,y,z,w)
    Eigen::Matrix <simFloat, 3, 1> linVelocity;
    Eigen::Matrix <simFloat, 3, 1> angVelocity;
    const static Eigen::Quaternion< simFloat > nwuToNed(0,1,0,0);

// Turn the propellers
    for (int motorIdx = 0; motorIdx < 4; ++motorIdx){
         if(simSetJointTargetVelocity(_handleOfJoint[motorIdx], 17.0)==-1){
            simSetLastError( simGetObjectName(_handleOfJoint[motorIdx]), "Error applying velocity.");
          }
       }


    if(simGetObjectPosition(_handleOfCoM, -1, position.data())!=-1 &&
            simGetObjectQuaternion(_handleOfCoM, -1, orientation.coeffs().data())!=-1 &&
            simGetObjectVelocity(_handleOfCoM, linVelocity.data(), angVelocity.data())!=-1){


        position = nwuToNed*position;
        linVelocity = nwuToNed*linVelocity;
        angVelocity = orientation.conjugate()*angVelocity; // Express angular velocity in body frame
        orientation = nwuToNed*orientation; // change world frame to north-east-down
        // use only positive w (not necessary)
        if (orientation.w()<0){
            orientation.coeffs() *=-1;
        }


        const simFloat currentSimulationTime = simGetSimulationTime();


        //if ((currentSimulationTime-_lastPublishedStatusTime) >= 1.0/_ObjStatusFrequency){



	        // Fill the status msg
			geometry_msgs::PoseStamped msgPose;
			msgPose.header.stamp = now;
			msgPose.pose.position.x = position[0];
			msgPose.pose.position.y = position[1];
			msgPose.pose.position.z = position[2];
			msgPose.pose.orientation.w = orientation.w();
			msgPose.pose.orientation.x = orientation.x();
			msgPose.pose.orientation.y = orientation.y();
			msgPose.pose.orientation.z = orientation.z();
		   _pubPose.publish(msgPose);


			geometry_msgs::TwistStamped msgTwist;
			msgTwist.header.stamp = now;
			msgTwist.twist.linear.x = linVelocity[0];
			msgTwist.twist.linear.y = linVelocity[1];
			msgTwist.twist.linear.z = linVelocity[2];
			msgTwist.twist.angular.x = angVelocity[0];
			msgTwist.twist.angular.y = angVelocity[1];
			msgTwist.twist.angular.z = angVelocity[2];
			_pubTwist.publish(msgTwist);

			_lastPublishedStatusTime = currentSimulationTime;

        //}

    }





     //Do the control
     if ((now-_lastReceivedCmdTime).toSec() > 0.1){

    	 simResetDynamicObject(_associatedObjectID);
         simSetObjectIntParameter( _associatedObjectID,3003, 1);
         simSetObjectIntParameter( _associatedObjectID,3004, 0); // Set the shape relative to the joint as NOT RESPONSABLE

         if ((now-_lastPrintedMsg).toSec() >= 1){

             std::stringstream ss;
             //ss << "- [" << _associatedObjectName << "] No command received since more than " << (now-_lastReceivedCmdTime).toSec() << "s!" << std::endl;
             ss <<"- [" << _associatedObjectName << "]  No commands: Static mode activated." << std::endl;
             simAddStatusbarMessage(ss.str().c_str());
             //ConsoleHandler::printInConsole(ss);
             _lastPrintedMsg = now;

         }
         return;
     } else

     {
         std::stringstream ss;
         //simResetDynamicObject(_associatedObjectID);
    	 simSetObjectIntParameter( _associatedObjectID,3003, 0);
    	 ss << " Receiving and applying commands from ROS." << std::endl;
         ConsoleHandler::printInConsole(ss);

     }


    // Command w.r.t. the quadrotor frame
    Eigen::Matrix< simFloat, 3, 1> ForceCommand(0.0 , 0.0 , (simFloat)_Commands[3]);
    Eigen::Matrix< simFloat, 3, 1> TorqueCommand ((simFloat)_Commands[0] , (simFloat)_Commands[1] , (simFloat)_Commands[2]);

    // Command w.r.t. the world frame
    //Eigen::Matrix< simFloat, 3, 1> worldForce = orientation*ForceCommand;
    //Eigen::Matrix< simFloat, 3, 1> worldTorqueCommand = orientation*TorqueCommand;


    // Print information
    //std::cout << "FORCE BOSY:"<< std::endl;
   // std::cout <<"x: "<< ForceCommand[0] <<" y: " << ForceCommand[1] <<"z: "<< ForceCommand[2]  << std::endl;

    //std::cout << "TORQUE BODY:"<< std::endl;
    //std::cout <<"x: "<< TorqueCommand[0] <<" y: " << TorqueCommand[1] <<"z: "<< TorqueCommand[2]  << std::endl;

    const Eigen::Matrix< simFloat, 3, 1> worldTorqueCommand = nwuToNed*orientation*TorqueCommand; //rotate torque to world frame
    const Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*orientation*ForceCommand;





//std::cout << "WORLD FORCE APPLIED:"<< std::endl;
//std::cout <<"x: "<< worldTorqueCommand[0] <<" y: " << worldTorqueCommand[1] <<"z: "<< worldTorqueCommand[2]  << std::endl;




    // Apply force and torque to the quadrotor
   if(simAddForceAndTorque(_associatedObjectID,worldForce.data() ,worldTorqueCommand.data() )==-1){
          simSetLastError( _associatedObjectName.c_str(), "Error applying force.");
      }
 //  else
//   {
//       for (uint motorIdx = 0; motorIdx < 4; ++motorIdx){
//          if(simSetJointTargetVelocity(_handleOfJoint[motorIdx], 2.5*_Commands[3])==-1){
//             simSetLastError( simGetObjectName(_handleOfJoint[motorIdx]), "Error applying velocity.");
//           }
//        }
//     }






//    if ((now-_lastReceivedCmdTime).toSec() > 0.1){
//        //_tkMotorCommands = std::vector<double>(4,0);
//        // worldTorqueCommand = 0.0;
//       // _tkCommands.roll = 0.0;
//       // _tkCommands.yaw = 0.0;
//        //_tkCommands.thrust = _quadrotorMass*9.8*0.8;
//
//
//    	_Commands[0]= 0;
//    	_Commands[1]= 0;
//     	_Commands[2]= 0;
//    	_Commands[3]= _quadrotorMass*9.81;
//
//        if ((now-_lastPrintedMsg).toSec() >= 1){
//            std::stringstream ss;
//            ss << "- [" << _associatedObjectName << "] No command received since more than " << (now-_lastReceivedCmdTime).toSec() << "s!" << std::endl;
//            simAddStatusbarMessage(ss.str().c_str());
//            ConsoleHandler::printInConsole(ss);
//            _lastPrintedMsg = now;
//        }
//    }



    }



void QuadrotorHandler::_initialize(){
    if (_initialized)
        return;

    // get some data from the main object
    std::vector<unsigned char> developerCustomData;
    getDeveloperCustomData(developerCustomData);

    // 2. From that retrieved data, try to extract sub-data with the IMU_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;
    std::stringstream ss;

    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::QUADROTOR_DATA_MAIN,tempMainData)){
    	int temp_freq = CAccess::pop_int(tempMainData);
    	if (temp_freq > 0){
    		_ObjStatusFrequency = temp_freq;
    		//std::cout << "Found Twist target in " << _associatedObjectName << ". Frequency publisher: " << _ObjTwistFrequency << std::endl;

    		ss << "- [Quadrotor] in '" << _associatedObjectName << "'. Frequency publisher: " << _ObjStatusFrequency << "." << std::endl;
    		ConsoleHandler::printInConsole(ss);
      	} else {
      		//std::cout << "Found Twist target in " << _associatedObjectName << " at the simulation frequency. " << std::endl;

      		ss << "- [Quadrotor] in '" << _associatedObjectName << "'. Frequency publisher: Simulation frequency. " << std::endl;
    	    ConsoleHandler::printInConsole(ss);
      	}






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


    // Subscriber of the command
    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');

    try
    	{
             _sub = _nh.subscribe(objectName+"/command", 1000, &QuadrotorHandler::CommandsCallback, this);
        }

    catch (ros::Exception &e)
    	{
            std::stringstream ss(e.what());
            ss << std::endl;
            ConsoleHandler::printInConsole(ss);
        }



    ConsoleHandler::printInConsole(ss);
    _initialized=true;
}

void QuadrotorHandler::CommandsCallback(const sensor_msgs::JointStateConstPtr& msg){
    if ( msg->effort.size() == 4){

    	_Commands = msg->effort;
    	_lastReceivedCmdTime = ros::Time::now();

    } else {
        simSetLastError( _associatedObjectName.c_str(), "Received wrong command size.");

    }
}



PLUGINLIB_EXPORT_CLASS(QuadrotorHandler, GenericObjectHandler)




