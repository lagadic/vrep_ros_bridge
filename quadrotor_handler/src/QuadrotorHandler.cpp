
#include <pluginlib/class_list_macros.h>
#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <quadrotor_handler/QuadrotorHandler.h>

#include <v_repLib.h>

//#include <telekyb_msgs/TKState.h>
#include <sensor_msgs/Imu.h>

#include <eigen3/Eigen/Geometry>

#include <vrep_ros_plugin/access.h>
#include <vrep_ros_plugin/ConsoleHandler.h>

QuadrotorHandler::QuadrotorHandler() : GenericObjectHandler(),
    _torqueToForceRatio(0.0),
    _handleOfCoM(-1),
    _quadrotorMass(0.8),
    _lastReceivedCmdTime(ros::Time::now())

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

    // Publisher of the IMU
    _pubIMU = _nh.advertise<sensor_msgs::Imu>(objectName+"/IMU", 1000);


}



void QuadrotorHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    ros::Time now = ros::Time::now();

    Eigen::Matrix<simFloat, 3, 1> position;
    Eigen::Quaternion< simFloat > orientation; //(x,y,z,w)
    Eigen::Matrix<simFloat, 3, 1> linVelocity;
    Eigen::Matrix<simFloat, 3, 1> angVelocity;
    const static Eigen::Quaternion< simFloat > nwuToNed(0,1,0,0);

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

//        // Fill the status msg
//        telekyb_msgs::TKState msg;
//        msg.header.stamp = now;
//        msg.pose.position.x = position[0];
//        msg.pose.position.y = position[1];
//        msg.pose.position.z = position[2];
//        msg.pose.orientation.w = orientation.w();
//        msg.pose.orientation.x = orientation.x();
//        msg.pose.orientation.y = orientation.y();
//        msg.pose.orientation.z = orientation.z();
//        msg.twist.linear.x = linVelocity[0];
//        msg.twist.linear.y = linVelocity[1];
//        msg.twist.linear.z = linVelocity[2];
//        msg.twist.angular.x = angVelocity[0];
//        msg.twist.angular.y = angVelocity[1];
//        msg.twist.angular.z = angVelocity[2];
//        _pub.publish(msg);

    }


    // Do the control
     if ((now-_lastReceivedCmdTime).toSec() > 0.1){
         if ((now-_lastPrintedMsg).toSec() >= 1){
             std::stringstream ss;
             ss << "- [" << _associatedObjectName << "] No command received since more than " << (now-_lastReceivedCmdTime).toSec() << "s!" << std::endl;
             simAddStatusbarMessage(ss.str().c_str());
             ConsoleHandler::printInConsole(ss);
             _lastPrintedMsg = now;
         }
         return;
     }


    // Command w.r.t. the quadrotor frame
    Eigen::Matrix< simFloat, 3, 1> ForceCommand(0.0 , 0.0 , _lastReceivedCmd.effort[3]);
    Eigen::Matrix< simFloat, 3, 1> TorqueCommand (_lastReceivedCmd.effort[0] , _lastReceivedCmd.effort[1] , _lastReceivedCmd.effort[2]);

    // Command w.r.t. the world frame
    Eigen::Matrix< simFloat, 3, 1> worldForce = nwuToNed*orientation*ForceCommand;
    Eigen::Matrix< simFloat, 3, 1> worldTorqueCommand = nwuToNed*orientation*TorqueCommand;


    // Apply force and torque to the quadrotor
     if(simAddForceAndTorque(_associatedObjectID,worldForce.data() ,worldTorqueCommand.data() )==-1){
            simSetLastError( _associatedObjectName.c_str(), "Error applying force.");
        }

//    }


    Eigen::Matrix<simFloat, 3, 1> VectorG (0.0, 0.0, 9.8);
    VectorG = orientation.conjugate()*VectorG; // Express angular velocity in body frame

    // Fill the IMU msg
     sensor_msgs::Imu msg1;
     msg1.header.stamp = ros::Time::now();
     msg1.angular_velocity.x = angVelocity[0];
     msg1.angular_velocity.y = angVelocity[1];
     msg1.angular_velocity.z = angVelocity[2];
     msg1.linear_acceleration.x = ForceCommand[0]/_quadrotorMass + VectorG[0];
     msg1.linear_acceleration.y = ForceCommand[1]/_quadrotorMass + VectorG[1];
     msg1.linear_acceleration.z = ForceCommand[2]/_quadrotorMass + VectorG[2];
     _pubIMU.publish(msg1); // Publish the Imu message in ROS

    if ((now-_lastReceivedCmdTime).toSec() > 0.1){
        //_tkMotorCommands = std::vector<double>(4,0);
        //_tkCommands.pitch = 0.0;
       // _tkCommands.roll = 0.0;
       // _tkCommands.yaw = 0.0;
        //_tkCommands.thrust = _quadrotorMass*9.8*0.8;
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


void QuadrotorHandler::_initialize(){
    if (_initialized)
        return;

    // get some data from the main object
    std::vector<unsigned char> developerCustomData;
    getDeveloperCustomData(developerCustomData);

    // 2. From that retrieved data, try to extract sub-data with the IMU_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;
    std::stringstream ss;

    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::QUADROTOR_DATA_TF_RATIO,tempMainData)){
        _torqueToForceRatio=CAccess::pop_float(tempMainData);
        ss << "- [" << _associatedObjectName << "] Setting torque to force ratio to: " << _torqueToForceRatio << "." << std::endl;
    } else {
        _torqueToForceRatio = 0.01738;
        ss << "- [" << _associatedObjectName << "] Torque to force ratio not specified. using " << _torqueToForceRatio << " as default."  << std::endl;
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

    	_lastReceivedCmd = *msg;
    	_lastReceivedCmdTime = ros::Time::now();

    } else {
        simSetLastError( _associatedObjectName.c_str(), "Received wrong command size.");

    }
}



PLUGINLIB_EXPORT_CLASS(QuadrotorHandler, GenericObjectHandler)




