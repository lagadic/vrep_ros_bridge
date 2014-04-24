
#include <pluginlib/class_list_macros.h>

#include <imu_handler/ImuHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>


ImuHandler::ImuHandler() : GenericObjectHandler(),
    _handleOfMass(-1),
    _mass(1.0),
    _handleOfForceSensor(-1),
    _lastPublishedImu(0.0),
    _acquisitionFrequency(-1.0)
   // _forceFilterCutoff(-1.0)
{
}

ImuHandler::~ImuHandler(){
}

unsigned int ImuHandler::getObjectType() const {
    return CustomDataHeaders::IMU_DATA_MAIN;
}

void ImuHandler::synchronize(){
//    // We update ImuHandler's data from its associated scene object custom data:
//    // 1. Get all the developer data attached to the associated scene object (this is custom data added by the developer):
//    int buffSize=simGetObjectCustomDataLength(_associatedObjectID,DEVELOPER_DATA_HEADER);
//    char* datBuff=new char[buffSize];
//    simGetObjectCustomData(_associatedObjectID,DEVELOPER_DATA_HEADER,datBuff);
//    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
//    delete[] datBuff;
//    // 2. From that retrieved data, try to extract sub-data with the QUADROTOR_DATA_MAIN tag:
//    std::vector<unsigned char> tempMainData;
//    if (CAccess::extractSerializationData(developerCustomData,QUADROTOR_DATA_MAIN,tempMainData))
//    { // Yes, the tag is there. For now we only have to synchronize _maxVelocity:
//    }


//    int buffSize=simGetObjectCustomDataLength(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER);
//    char* datBuff=new char[buffSize];
//    simGetObjectCustomData(_associatedObjectID, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
//    std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
//    delete[] datBuff;

//    std::vector<unsigned char> developerCustomData;
//    getDeveloperCustomData(developerCustomData);

    // 2. From that retrieved data, try to extract sub-data with the IMU_DATA_MAIN tag:
//    std::vector<unsigned char> tempMainData;

//    if (CAccess::extractSerializationData(developerCustomData,IMU_DATA_FREQ,tempMainData)){ // Yes, the tag is there. For now we only have to synchronize _acquisitionFrequency:
//        _acquisitionFrequency=CAccess::pop_int(tempMainData);
//         std::cout << "Imu frequency: " << _acquisitionFrequency << "." << std::endl;
//    } else {
//        std::cout << "Imu frequency not specified. Using simulation frequency as default." << std::endl;
//    }


    // Remove # chars for compatibility with ROS
    _associatedObjectName = simGetObjectName(_associatedObjectID);
    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');
    _pub = _nh.advertise<sensor_msgs::Imu>(objectName, 1000);

}

void ImuHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    ros::Time now = ros::Time::now();

    const simFloat currentSimulationTime = simGetSimulationTime();

    if ( ((currentSimulationTime - _lastPublishedImu) >= 1.0/_acquisitionFrequency) ){


		Eigen::Quaternion< simFloat > orientation; //(x,y,z,w)
		Eigen::Matrix<simFloat, 3, 1> angVelocity;
		simFloat force[3];

		if(simGetObjectQuaternion(_handleOfMass, -1, orientation.coeffs().data())!=-1 &&
				simGetObjectVelocity(_handleOfMass, NULL, angVelocity.data())!=-1 &&
				simReadForceSensor(_handleOfForceSensor, force, NULL)){

			angVelocity = orientation.conjugate()*angVelocity; // Express angular velocity in body frame

/*			// Filter force
			if (_forceFilterCutoff>0){
                for (uint i = 0; i<3; ++i){
                    double output;
                    _forceFilter[i]->step(std::vector<double>(1,force[i]), output);
                    force[i] = (simFloat)output;
                }
			}
*/

			// Fill the imu msg
			sensor_msgs::Imu msg;

			msg.header.stamp = ros::Time::now();
			msg.angular_velocity.x = angVelocity[0];
			msg.angular_velocity.y = angVelocity[1];
			msg.angular_velocity.z = angVelocity[2];
			msg.linear_acceleration.x = force[0]/_mass;
			msg.linear_acceleration.y = force[1]/_mass;
			msg.linear_acceleration.z = force[2]/_mass;
			_pub.publish(msg); // Publish the Imu message in ROS

			_lastPublishedImu = currentSimulationTime;
		}

    }


}


void ImuHandler::_initialize(){
    if (_initialized)
        return;

    // get some data from the main object
    std::vector<unsigned char> developerCustomData;
    getDeveloperCustomData(developerCustomData);

    // 2. From that retrieved data, try to extract sub-data with the IMU_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;
    std::stringstream ss;

    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::IMU_DATA_FREQ,tempMainData)){
        _acquisitionFrequency=CAccess::pop_float(tempMainData);
        if (_acquisitionFrequency > 0){
            ss << "- [" << _associatedObjectName << "] Frequency publisher: " << _acquisitionFrequency << "." << std::endl;
        } else {
            ss << "- [" << _associatedObjectName << "] Frequency publisher: simulation frequency."  << std::endl;
        }
    } else {
        ss << "- [" << _associatedObjectName << "] Imu frequency publisher not specified. using simulation frequency as default."  << std::endl;
    }


    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::IMU_DATA_CUTOFF,tempMainData)){
        _forceFilterCutoff=CAccess::pop_float(tempMainData);
        ss << "- [" << _associatedObjectName;
        if (_forceFilterCutoff>0){
            ss << "] Force will be filtered at " << _forceFilterCutoff << " Hz." << std::endl;
        } else {
            ss << "] Force filter cutoff frequency set negative. Force will not be filtered." << std::endl;
        }
    } else {
        _forceFilterCutoff=-1.0;
        ss << "- [" << _associatedObjectName << "] Force filter cutoff frequency not specified. Force will not be filtered." << std::endl;
    }


    // search associated objects
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
        int buffSize=simGetObjectCustomDataLength(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);
        if (buffSize!=0) { // Yes there is some custom data written by us (the developer with the DEVELOPER_DATA_HEADER header)
            char* datBuff=new char[buffSize];
            simGetObjectCustomData(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
            std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
            delete[] datBuff;
            // b. From that retrieved data, try to extract sub-data with the searched tags:
            std::vector<unsigned char> tempMainData;

            if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::IMU_DATA_MASS,tempMainData)){
                _handleOfMass=objHandle;
                simGetObjectFloatParameter(_handleOfMass, 3005, &_mass);

            } else if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::IMU_DATA_FORCE,tempMainData)){
                _handleOfForceSensor=objHandle; // We found the IMU_DATA_FORCE tag. This is the Force sensor!
                ss << "- [" << _associatedObjectName << "] Found ForceSensor. The acceleration will be computed and published." << std::endl;
            }

        }
    }


  /*  // initialize acceleration filter
    if(_forceFilterCutoff>0.0){
        const double filterSampleTime = _acquisitionFrequency > 0 ? 1.0/_acquisitionFrequency : (double)simGetSimulationTimeStep();
        for(uint i=0;i<3;++i){
            _forceFilter[i] = new telekyb::IIRFilter(telekyb::IIRLowPass(), 2.0*M_PI*_forceFilterCutoff, 1.0, filterSampleTime);
        }
    }
*/
    ConsoleHandler::printInConsole(ss);

    _lastPublishedImu = -1.0;

    _initialized=true;
}

bool ImuHandler::endOfSimulation(){
 /*   if(_forceFilterCutoff>0.0){
        for(uint i=0;i<3;++i){
            delete _forceFilter[i];
        }
    }
    */
    _initialized=false;
    return(false); // We don't want this object automatically destroyed at the end of simulation
}

PLUGINLIB_EXPORT_CLASS(ImuHandler, GenericObjectHandler)
