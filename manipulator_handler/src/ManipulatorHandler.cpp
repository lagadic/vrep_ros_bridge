
#include <pluginlib/class_list_macros.h>

#include <manipulator_handler/ManipulatorHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>
#include <vrep_ros_plugin/v_repExtRosBridge.h>


#include <vrep_ros_plugin/ConsoleHandler.h>

#include <rosconsole/macros_generated.h>

ManipulatorHandler::ManipulatorHandler() : GenericObjectHandler(),
_acquisitionFrequency(-1.0),
_handleOfJoints(0),
_numJoints(0),
_defaultModeCtrl(CustomDataHeaders::TF_POSITION),
_axle_lenght (0.331),
_mb_radius(0.0970){
}

ManipulatorHandler::~ManipulatorHandler(){
}

unsigned int ManipulatorHandler::getObjectType() const {
    return CustomDataHeaders::MANIPULATOR_DATA_MAIN;
}

void ManipulatorHandler::synchronize(){

    // Remove # chars for compatibility with ROS
    _associatedObjectName = simGetObjectName(_associatedObjectID);
    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');
    _pub = _nh.advertise<sensor_msgs::JointState>(objectName+"/jointStatus", 1000);


}

void ManipulatorHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    ros::Time now = ros::Time::now();

    const simFloat currentSimulationTime = simGetSimulationTime();

    if ( ((currentSimulationTime - _lastPublishedStatus) >= 1.0/_acquisitionFrequency) ){

        sensor_msgs::JointState msg;
        msg.header.stamp = now;
        const unsigned int ndofs = _handleOfJoints.size();
        msg.name.resize(ndofs);
        msg.position.resize(ndofs);
        msg.velocity.resize(ndofs);
        for (uint jointIdx = 0; jointIdx < ndofs; ++jointIdx){
            msg.name[jointIdx] = _jointNames[jointIdx];
            simFloat temp;
            simGetJointPosition(_handleOfJoints[jointIdx],&temp);
            msg.position[jointIdx] = temp;
            simGetObjectFloatParameter(_handleOfJoints[jointIdx], 2012, &temp);
            msg.velocity[jointIdx] = temp;
        }

        _pub.publish(msg); // Publish the Joint status message in ROS
        _lastPublishedStatus = currentSimulationTime;

    }

    // Do the control

    // Set all joint velocities to zero if we haven't received any commands for a long time
    if (currentSimulationTime-_lastReceivedCmdTime > 0.1){
        for(uint jointIdx = 0; jointIdx < _handleOfJoints.size(); ++jointIdx){
            simSetJointTargetVelocity(_handleOfJoints[jointIdx], 0.0);
        }
        if (_lastReceivedCmdTime > 0 && (now-_lastPrintedMsg).toSec() >= 1){
            std::stringstream ss;
            ss << "- [" << _associatedObjectName << "] No command received since more than " << currentSimulationTime -_lastReceivedCmdTime << "s!" << std::endl;
            simAddStatusbarMessage(ss.str().c_str());
            //ConsoleHandler::printInConsole(ss);
            _lastPrintedMsg = now;
        }
        return;
    }


    for(uint jointIdx = 0; jointIdx < _handleOfJoints.size(); ++jointIdx){
        if (_jointCtrlMode[jointIdx] == CustomDataHeaders::TF_POSITION){

            ROS_INFO("Apply new position to joint %s: %f.", _jointNames[jointIdx].c_str(), _lastReceivedCmd.position[jointIdx]);
            //_tempjoint = _tempjoint+0.001;
            if(simSetJointTargetPosition(_handleOfJoints[jointIdx], _lastReceivedCmd.position[jointIdx])==-1)
                // if(simSetJointTargetPosition(_handleOfJoints[jointIdx], _tempjoint)==-1)
            {
                std::stringstream ss;
                ss << "- [" << _associatedObjectName << "] Error setting position for joint "<<
                        jointIdx <<" (" << _jointNames[jointIdx] <<")." <<std::endl;
                ConsoleHandler::printInConsole(ss);
            }



        } else if (_jointCtrlMode[jointIdx] == CustomDataHeaders::TF_VELOCITY ||_jointCtrlMode[jointIdx] == CustomDataHeaders::MOT_VELOCITY ){

            ROS_DEBUG("Apply new velocity to  joint %s: %f.", _jointNames[jointIdx].c_str(), _lastReceivedCmd.velocity[jointIdx]);
            if(simSetJointTargetVelocity(_handleOfJoints[jointIdx], _lastReceivedCmd.velocity[jointIdx])==-1){
                std::stringstream ss;
                ss << "- [" << _associatedObjectName << "] Error setting velocity for joint "<<
                        jointIdx <<" (" << _jointNames[jointIdx] <<")." <<std::endl;
                ConsoleHandler::printInConsole(ss);
            }

        } else if (_jointCtrlMode[jointIdx] == CustomDataHeaders::TF_EFFORT ){

            ROS_DEBUG("Apply new torque/force to joint %s: %f.", _jointNames[jointIdx].c_str(), _lastReceivedCmd.effort[jointIdx]);
            if(simSetJointForce(_handleOfJoints[jointIdx], _lastReceivedCmd.effort[jointIdx])==-1){
                std::stringstream ss;
                ss << "- [" << _associatedObjectName << "] Error setting torque or force for joint "<<
                        jointIdx <<" (" << _jointNames[jointIdx] <<")." <<std::endl;
                ConsoleHandler::printInConsole(ss);
            }

        } else if (_jointCtrlMode[jointIdx] == CustomDataHeaders::PASSIVE_MODE){

            ROS_DEBUG("Apply new position to joint %s: %f.", _jointNames[jointIdx].c_str(), _lastReceivedCmd.position[jointIdx]);
            //_tempjoint = _tempjoint+0.001;
            if(simSetJointPosition(_handleOfJoints[jointIdx], _lastReceivedCmd.position[jointIdx])==-1){
                std::stringstream ss;
                ss << "- [" << _associatedObjectName << "] Error setting position for joint "<<
                        jointIdx <<" (" << _jointNames[jointIdx] <<")." <<std::endl;
                ConsoleHandler::printInConsole(ss);
            }


        } else if (_jointCtrlMode[jointIdx] == CustomDataHeaders::PASSIVE_MODE_VELOCITY){

            ROS_DEBUG("Apply new velocity to joint %s: %f.", _jointNames[jointIdx].c_str(), _lastReceivedCmd.velocity[jointIdx]);
            //_tempjoint = _tempjoint+0.001;

             float cur_pos;
             simGetJointPosition(_handleOfJoints[jointIdx], &cur_pos);
             float pos = cur_pos + _lastReceivedCmd.velocity[jointIdx] *  simGetSimulationTimeStep();

            if(simSetJointPosition(_handleOfJoints[jointIdx], pos)==-1){
                std::stringstream ss;
                ss << "- [" << _associatedObjectName << "] Error setting velocity for joint "<<
                        jointIdx <<" (" << _jointNames[jointIdx] <<")." <<std::endl;
                ConsoleHandler::printInConsole(ss);
            }

    }




    }

}


void ManipulatorHandler::_initialize(){
    if (_initialized)
        return;

    _numJoints = 0;

    // get some data from the main object
    std::vector<unsigned char> developerCustomData;
    getDeveloperCustomData(developerCustomData);

    std::vector<unsigned char> tempMainData;
    std::stringstream ss;
    std::string namectrlmode;

    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::MANIPULATOR_DATA_FREQ,tempMainData)){
        _acquisitionFrequency=CAccess::pop_float(tempMainData);
        if (_acquisitionFrequency > 0){
            ss << "- [" << _associatedObjectName << "] Publisher frequency: " << _acquisitionFrequency << "." << std::endl;
        } else {
            ss << "- [" << _associatedObjectName << "] Publisher frequency: simulation frequency."  << std::endl;
        }
    } else {
        ss << "- [" << _associatedObjectName << "] Joint status publisher frequency not specified. using simulation frequency as default."  << std::endl;
    }

    // We check if a default control mode is defined:
    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::MANIPULATOR_DATA_CTRL_MODE,tempMainData)){
        const int ctrlMode = CAccess::pop_int(tempMainData);

        ss << "- [" << _associatedObjectName << "] A Default Control Mode is defined: ";

        if (ctrlMode == (int)(CustomDataHeaders::TF_POSITION)){
            _defaultModeCtrl = CustomDataHeaders::TF_POSITION;
            ss << "Torque/Force control in POSITION (PID controller)." << std::endl;
            namectrlmode ="Torque/Force control in POSITION (PID controller)." ;

        } else if (ctrlMode == (int)(CustomDataHeaders::TF_VELOCITY)){
            _defaultModeCtrl = CustomDataHeaders::TF_VELOCITY;
            ss << "Torque/Force control in VELOCITY." << std::endl;
            namectrlmode ="Torque/Force control in VELOCITY." ;

        } else if (ctrlMode == (int)(CustomDataHeaders::TF_EFFORT)){
            _defaultModeCtrl = CustomDataHeaders::TF_EFFORT;
            ss << "Torque/Force control." << std::endl;
            namectrlmode ="Torque/Force control." ;

        } else if (ctrlMode == (int)(CustomDataHeaders::MOT_VELOCITY)){
            _defaultModeCtrl = CustomDataHeaders::MOT_VELOCITY;
            ss << "Motion control in VELOCITY." << std::endl;
            namectrlmode = "Motion control in VELOCITY." ;

        } else if (ctrlMode == (int)(CustomDataHeaders::PASSIVE_MODE)){
            _defaultModeCtrl = CustomDataHeaders::PASSIVE_MODE;
            ss << " Using Position control in Passive Mode." << std::endl;
            namectrlmode = "Position control in Passive Mode." ;
        } else if (ctrlMode == (int)(CustomDataHeaders::PASSIVE_MODE_VELOCITY)){
            _defaultModeCtrl = CustomDataHeaders::PASSIVE_MODE_VELOCITY;
            ss << " Using velocity control in Passive Mode." << std::endl;
            namectrlmode = " Velocity control in Passive Mode." ;
        } else {
            _defaultModeCtrl = CustomDataHeaders::TF_POSITION;
            ss << " Invalid control mode specified. Using POSITION as default." << std::endl;
            namectrlmode = "Torque/Force control in POSITION (PID controller)." ;
        }



    } else {
        _defaultModeCtrl = CustomDataHeaders::TF_POSITION;
        ss << " Any control is mode specified. Using Torque/Force control in POSITION (PID controller) as default." << std::endl;
        namectrlmode = "Torque/Force control in POSITION (PID controller)."  ;
    }





    // search associated objects
    std::vector<int> toExplore;
    toExplore.push_back(_associatedObjectID); // We start exploration with the base of the manipulator
    while (toExplore.size()!=0)
    {
        int objHandle=toExplore[toExplore.size()-1];
        toExplore.pop_back();
        // 1. Add this object's children to the list to explore:
        int index=0;
        int childHandle=simGetObjectChild(objHandle,index++);
        ROS_DEBUG("__START___");
        while (childHandle!=-1) {
            toExplore.push_back(childHandle);
            ROS_DEBUG("Adding %s to exploration list.", simGetObjectName(childHandle));
            childHandle=simGetObjectChild(objHandle,index++);
        }
        ROS_DEBUG("__END___");
        // 2. Now check if this object has one of the tags we are looking for:
        // a. Get all the developer data attached to this scene object (this is custom data added by the developer):
        int buffSize=simGetObjectCustomDataLength(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);
        if (1) { // Yes there is some custom data written by us (the developer with the DEVELOPER_DATA_HEADER header)
            char* datBuff=new char[buffSize];
            simGetObjectCustomData(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
            std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
            delete[] datBuff;
            // b. From that retrieved data, try to extract sub-data with the searched tags:
            std::vector<unsigned char> tempMainData;


            //if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::MANIPULATOR_DATA_JOINT,tempMainData))
            int ctrlMode = -1;
            if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::MANIPULATOR_DATA_CTRL_MODE,tempMainData))
                ctrlMode = CAccess::pop_int(tempMainData);

            // TODO: add support for spherical joints (they are always passive)
            if ((simGetJointType(objHandle) == sim_joint_revolute_subtype || simGetJointType(objHandle) == sim_joint_prismatic_subtype)
                    && ctrlMode != (int)(CustomDataHeaders::IGNORE_MODE)){

                unsigned int  jointID = _numJoints;
                _numJoints++;
                ROS_DEBUG("Found %s. Id: %d", simGetObjectName(objHandle), jointID);
                if (_handleOfJoints.size() < jointID+1){
                    _handleOfJoints.resize(jointID+1);
                    _jointNames.resize(jointID+1);
                    _jointCtrlMode.resize(jointID+1);
                }
                _handleOfJoints[jointID]=objHandle;
                _jointNames[jointID]=simGetObjectName(objHandle);
                ss << "- [" << simGetObjectName(objHandle) << "] Found Joint.";

                if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::MANIPULATOR_DATA_CTRL_MODE,tempMainData)){
                    const int ctrlMode = CAccess::pop_int(tempMainData);

                    if (ctrlMode == (int)(CustomDataHeaders::TF_POSITION)){
                        _jointCtrlMode[jointID] = CustomDataHeaders::TF_POSITION;
                        ss << " Using Torque/Force control in POSITION (PID controller)." << std::endl;

                    } else if (ctrlMode == (int)(CustomDataHeaders::TF_VELOCITY)){
                        _jointCtrlMode[jointID] = CustomDataHeaders::TF_VELOCITY;
                        ss << " Using Torque/Force control in VELOCITY." << std::endl;

                    } else if (ctrlMode == (int)(CustomDataHeaders::TF_EFFORT)){
                        _jointCtrlMode[jointID] = CustomDataHeaders::TF_EFFORT;
                        ss << "Torque/Force control." << std::endl;

                    } else if (ctrlMode == (int)(CustomDataHeaders::MOT_VELOCITY)){
                        _jointCtrlMode[jointID] = CustomDataHeaders::MOT_VELOCITY;
                        ss << " Using Motion control in VELOCITY." << std::endl;

                    } else if (ctrlMode == (int)(CustomDataHeaders::PASSIVE_MODE)){
                        _jointCtrlMode[jointID] = CustomDataHeaders::PASSIVE_MODE;
                        ss << " Using Position control in Passive Mode." << std::endl;

                    } else if (ctrlMode == (int)(CustomDataHeaders::PASSIVE_MODE_VELOCITY)){
                        _jointCtrlMode[jointID] = CustomDataHeaders::PASSIVE_MODE_VELOCITY;
                        ss << " Using velocity control in Passive Mode." << std::endl;

                    } else {
                        _jointCtrlMode[jointID] = _defaultModeCtrl;
                        ss << "  Using default control mode: " << namectrlmode << std::endl;
                    }

                } else {
                    _jointCtrlMode[jointID] = _defaultModeCtrl;
                    ss << " Using default control mode: "  << namectrlmode << std::endl;

                    //  _jointCtrlMode[jointID] = CustomDataHeaders::VELOCITY;
                    // ss << " Control mode not specified. Using VELOCITY as default."  << std::endl;

                }

                if (_jointCtrlMode[jointID] == CustomDataHeaders::TF_POSITION){
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_force, 0);
                    simSetObjectIntParameter(_handleOfJoints[jointID],2001,1);

                } else if (_jointCtrlMode[jointID] == CustomDataHeaders::TF_VELOCITY){
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_force, 0);
                    simSetObjectIntParameter(_handleOfJoints[jointID],2001,0);

                } else if (_jointCtrlMode[jointID] == CustomDataHeaders::TF_EFFORT){
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_force, 0);
                    simSetObjectIntParameter(_handleOfJoints[jointID],2001,0);

                } else if (_jointCtrlMode[jointID] == CustomDataHeaders::MOT_VELOCITY){

#if VREP_VERSION_MAJOR*10000+VREP_VERSION_MINOR*100+VREP_VERSION_PATCH <= 3*10000+2*100+0
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_motion, 0);
                    simSetBooleanParameter(sim_boolparam_joint_motion_handling_enabled,1);
#else
                    ss << " WARNING: " << namectrlmode << " is deprecated! Use Passive_mode_velocity instead." << std::endl;
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_motion_deprecated, 0);
                    simSetBooleanParameter(sim_boolparam_joint_motion_handling_enabled_deprecated,1);
#endif

                    int childHandle = simGetObjectChild(_handleOfJoints[jointID],0);
                    simSetObjectIntParameter( childHandle,3003, 1); // Set the shape relative to the joint as STATIC
                    simSetObjectIntParameter( childHandle,3004, 0); // Set the shape relative to the joint as NOT RESPONSABLE

                } else if (_jointCtrlMode[jointID] == CustomDataHeaders::PASSIVE_MODE){
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_passive, 0);
                    int childHandle = simGetObjectChild(_handleOfJoints[jointID],0);
                    simSetObjectIntParameter( childHandle,3003, 1); // Set the shape relative to the joint as STATIC
                    simSetObjectIntParameter( childHandle,3004, 0); // Set the shape relative to the joint as NOT RESPONSABLE

                } else if (_jointCtrlMode[jointID] == CustomDataHeaders::PASSIVE_MODE_VELOCITY){
                    simSetJointMode(_handleOfJoints[jointID], sim_jointmode_passive, 0);
                    int childHandle = simGetObjectChild(_handleOfJoints[jointID],0);
                    simSetObjectIntParameter( childHandle,3003, 1); // Set the shape relative to the joint as STATIC
                    simSetObjectIntParameter( childHandle,3004, 0); // Set the shape relative to the joint as NOT RESPONSABLE


                }

            } else 	{
                ROS_DEBUG("Object %s will be ignored as requested.", simGetObjectName(objHandle));
            }

        }
    }

    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');
    _sub = _nh.subscribe(objectName+"/jointCommand", 1, &ManipulatorHandler::jointCommandCallback, this);

    // Subscriber for mobile robots
    _subVelMob = _nh.subscribe("/cmd_vel", 1, &ManipulatorHandler::VelMobCommandCallback, this);

    ss << "- [" << _associatedObjectName << "] Initialization done." << std::endl;
    ConsoleHandler::printInConsole(ss);

    _lastPublishedStatus = -1.0;
    _lastReceivedCmdTime = -1.0;
    _lastPrintedMsg = ros::Time(-1.0);

    _initialized=true;
}


void ManipulatorHandler::jointCommandCallback(const sensor_msgs::JointStateConstPtr& msg){


    const unsigned int nDofs = _handleOfJoints.size();
    if (msg->position.size()!=nDofs || msg->velocity.size()!=nDofs || msg->effort.size()!=nDofs){
        simSetLastError( _associatedObjectName.c_str(), "Received wrong command size.");
        ROS_ERROR("Received wrong command size (%ld, %ld, %ld) for manipulator which has %d dofs.",
                msg->position.size(), msg->velocity.size(), msg->effort.size(), nDofs);

    } else {
        _lastReceivedCmd = *msg;
        _lastReceivedCmdTime = simGetSimulationTime();
    }
}


/// Callback for joint commands.
void ManipulatorHandler::VelMobCommandCallback(const geometry_msgs::TwistConstPtr& msg){

    geometry_msgs::Twist temp = *msg;

    _lastReceivedCmd.velocity.resize(2);
    _lastReceivedCmd.velocity[0]= (1/_mb_radius)*(temp.linear.x - _axle_lenght/2*temp.angular.z); // left
    _lastReceivedCmd.velocity[1]= (1/_mb_radius)*(temp.linear.x + _axle_lenght/2*temp.angular.z);  // rigth

    _lastReceivedCmdTime = simGetSimulationTime();

}



PLUGINLIB_EXPORT_CLASS(ManipulatorHandler, GenericObjectHandler)
