

#include "vrep_ros_plugin/access.h"
#include "v_repLib.h"

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <boost/lexical_cast.hpp>

CAccess::CAccess(){
}

CAccess::~CAccess(){
}

int CAccess::getDataLocationAndSize(const std::vector<unsigned char>& buffer,int dataName,int& theSize){
	int p=0;
	while (p<=int(buffer.size()-2*sizeof(int)))
	{
		const int currHead = *((int*)(buffer.data()+p));
		const int currSize = *((int*)(buffer.data()+p)+1);
		if (currHead==dataName){
		    theSize=currSize;
		    return p+2*sizeof(int);
		}
		p+=2*sizeof(int)+currSize;
//		for (int i=0;i<4;i++)
//			((unsigned char*)&n)[i]=buffer[p+i];
//		for (int i=0;i<4;i++)
//			((unsigned char*)&l)[i]=buffer[p+4+i];
//		if (n==dataName){
//			if (l!=0)
//			{
//				theSize=l;
//				return(p+8);
//			}
//			return(-1); // shouldn't happen if data saved correctly
//		}
//		p+=8+l;
	}
	return(-1); // data not present
}

bool CAccess::extractSerializationData(std::vector<unsigned char>& buffer, const int dataName, std::vector<unsigned char>& data){
//	data.clear();
	int dataSize;
	const int dataLocation = getDataLocationAndSize(buffer, dataName, dataSize);
	if (dataLocation!=-1)
	{
//		for (int i=0;i<l;i++)
//			data.push_back(buffer[p+i]);
	    data.resize(dataSize);
	    memcpy(data.data(), buffer.data()+dataLocation,dataSize);
		buffer.erase(buffer.begin()+dataLocation-2*sizeof(int),buffer.begin()+dataLocation+dataSize);
		return(true);
	}
	return(false); // that data was not present!
}

void CAccess::insertSerializationData(std::vector<unsigned char>& buffer,int dataName,const std::vector<unsigned char>& data){
	// 1. We remove the old data:
	std::vector<unsigned char> dummyBuffer;

	extractSerializationData(buffer,dataName,dummyBuffer);
	// 2. and insert the new data:
	// 2.1 data name:

	const unsigned int prevBuffSize = buffer.size();
	const unsigned int dataSize = data.size();
	buffer.resize(prevBuffSize+2*sizeof(int)+dataSize);
	int* prevBuffEnd = (int *)(buffer.data()+prevBuffSize);

	prevBuffEnd[0] = dataName;
	prevBuffEnd[1] = dataSize;
	memcpy((unsigned char *)(prevBuffEnd+2),data.data(),dataSize);

//	for (int i=0;i<4;i++)
//		buffer.push_back(((unsigned char*)&dataName)[i]);
//
//	std::cout << "bufferSize: " << buffer.size() << std::endl;
//
//	// 2.2 Data length:
//	int l=int(data.size());
//	for (int i=0;i<4;i++)
//		buffer.push_back(((unsigned char*)&l)[i]);
//	// 2.3 The data itself:
//	for (int i=0;i<l;i++)
//		buffer.push_back(data[i]);
}

void CAccess::push_int(std::vector<unsigned char>& buffer, const int data){
//	for (unsigned int i=0;i<sizeof(data);i++)
//		buffer.push_back(((unsigned char*)&data)[i]);
	buffer.resize(buffer.size()+sizeof(int));
	*((int*)(&(*buffer.end()))-1) = data;
}

void CAccess::push_float(std::vector<unsigned char>& buffer, const float data){
//	for (unsigned int i=0;i<sizeof(data);i++)
//		buffer.push_back(((unsigned char*)&data)[i]);
    buffer.resize(buffer.size()+sizeof(float));
    *((float*)(&(*buffer.end()))-1) = data;
}

void CAccess::push_float(std::vector<unsigned char>& buffer, const std::vector<float> data){
    buffer.resize(buffer.size()+sizeof(float)*data.size());
	float * buf = ((float*)(&(*buffer.end()))) - data.size();
	memcpy(buf, data.data(), sizeof(float)*data.size());
}

int CAccess::pop_int(std::vector<unsigned char>& buffer){
//	int retVal=0;
//	if (buffer.size()<sizeof(retVal))
//		return(retVal);
//	for (int i=int(sizeof(retVal))-1;i>=0;i--)
//	{
//		((unsigned char*)&retVal)[i]=buffer[buffer.size()-1];
//		buffer.pop_back();
//	}
//	return(retVal);
    if (buffer.size()<sizeof(int)){
          return 0;
    } else {
        const float retVal=*((int*)(&(*buffer.end()))-1);
        buffer.resize(buffer.size()-sizeof(int));
        return retVal;
    }
}

float CAccess::pop_float(std::vector<unsigned char>& buffer){
//	float retVal=0.0f;
//	if (buffer.size()<sizeof(retVal))
//		return(retVal);
//	for (int i=int(sizeof(retVal))-1;i>=0;i--)
//	{
//		((unsigned char*)&retVal)[i]=buffer[buffer.size()-1];
//		buffer.pop_back();
//	}
//	return(retVal);
    if (buffer.size()<sizeof(float)){
          return 0.0;
    } else {
        const float retVal=*((float*)(&(*buffer.end()))-1);
        buffer.resize(buffer.size()-sizeof(float));
        return retVal;
    }
}

std::vector<float> CAccess::pop_float(std::vector<unsigned char>& buffer, const unsigned int n){
	std::vector<float> out;
	if (buffer.size()>=sizeof(float)*n){
		out.resize(n);
		const float * buf = ((const float*)(&(*buffer.end()))) - n;
		memcpy(out.data(), buf, sizeof(float)*n);
		buffer.resize(buffer.size()-sizeof(float)*n);
	}
	return out;
}

void CustomDataHeaders::registerCustomDataHeaders(){


    simRegisterCustomLuaVariable("sim_ext_ros_bridge_developer_data_header", (boost::lexical_cast<std::string>(int(DEVELOPER_DATA_HEADER))).c_str());

    // Quadrotor defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_main", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_MAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_tk_data_main", (boost::lexical_cast<std::string>(int(QUADROTOR_TK_DATA_MAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_motor_0", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_MOTOR_0))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_motor_1", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_MOTOR_1))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_motor_2", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_MOTOR_2))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_motor_3", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_MOTOR_3))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_com", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_COM))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_tf_ratio", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_TF_RATIO))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_data_ctrl_mode", (boost::lexical_cast<std::string>(int(QUADROTOR_DATA_CTRL_MODE))).c_str());

    //Quadrotor ctrl modes
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_ctrl_mode_direct", (boost::lexical_cast<std::string>(int(DIRECT))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_quadrotor_ctrl_mode_internal", (boost::lexical_cast<std::string>(int(INTERNAL))).c_str());

    // IMU defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_imu_data_main", (boost::lexical_cast<std::string>(int(IMU_DATA_MAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_imu_data_mass", (boost::lexical_cast<std::string>(int(IMU_DATA_MASS))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_imu_data_force", (boost::lexical_cast<std::string>(int(IMU_DATA_FORCE))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_imu_data_freq", (boost::lexical_cast<std::string>(int(IMU_DATA_FREQ))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_imu_data_cutoff", (boost::lexical_cast<std::string>(int(IMU_DATA_CUTOFF))).c_str());

    // Force sensor defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_force_sensor_data_main", (boost::lexical_cast<std::string>(int(FORCE_SENSOR_DATA_MAIN))).c_str());

    // Camera defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_camera_data_main", (boost::lexical_cast<std::string>(int(CAMERA_DATA_MAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_camera_data_freq", (boost::lexical_cast<std::string>(int(CAMERA_DATA_FREQ))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_camera_data_rgb", (boost::lexical_cast<std::string>(int(CAMERA_DATA_RGB))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_camera_data_has_depth", (boost::lexical_cast<std::string>(int(CAMERA_DATA_HAS_DEPTH))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_camera_data_use_float", (boost::lexical_cast<std::string>(int(CAMERA_DATA_USE_FLOAT))).c_str());

    // Object Pose defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_obj_pose_data_main", (boost::lexical_cast<std::string>(int(OBJ_POSE_DATA_MAIN))).c_str());

    // Object twist defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_obj_twist_data_main", (boost::lexical_cast<std::string>(int(OBJ_TWIST_DATA_MAIN))).c_str());

    // Manipulator defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_data_main", (boost::lexical_cast<std::string>(int(MANIPULATOR_DATA_MAIN))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_data_freq", (boost::lexical_cast<std::string>(int(MANIPULATOR_DATA_FREQ))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_data_joint", (boost::lexical_cast<std::string>(int(MANIPULATOR_DATA_JOINT))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_data_ctrl_mode", (boost::lexical_cast<std::string>(int(MANIPULATOR_DATA_CTRL_MODE))).c_str());

    // Manipulator ctrl modes
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_TF_position", (boost::lexical_cast<std::string>(int(TF_POSITION))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_TF_velocity", (boost::lexical_cast<std::string>(int(TF_VELOCITY))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_TF_effort", (boost::lexical_cast<std::string>(int(TF_EFFORT))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_MOT_velocity", (boost::lexical_cast<std::string>(int(MOT_VELOCITY))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_Passive_mode", (boost::lexical_cast<std::string>(int(PASSIVE_MODE))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_Passive_mode_velocity", (boost::lexical_cast<std::string>(int(PASSIVE_MODE_VELOCITY))).c_str());
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_manipulator_ctrl_mode_Ignore_mode", (boost::lexical_cast<std::string>(int(IGNORE_MODE))).c_str());

    // Contact defines
    simRegisterCustomLuaVariable("sim_ext_ros_bridge_contact_data_main", (boost::lexical_cast<std::string>(int(CONTACT_DATA_MAIN))).c_str());

}




