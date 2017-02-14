
#include <pluginlib/class_list_macros.h>
#include <camera_handler/CameraHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <vrep_ros_plugin/ConsoleHandler.h>



CameraHandler::CameraHandler() : GenericObjectHandler(),
_acquisitionFrequency(30.0),
_lastPublishedImageTime(0.0),
_cameraIsRGB(true),
_cameraHasDepth(false),
_cameraIsFloat(false),
_it(_nh){
}

CameraHandler::~CameraHandler(){
}

unsigned int CameraHandler::getObjectType() const {
	return CustomDataHeaders::CAMERA_DATA_MAIN;
}

void CameraHandler::synchronize(){

	_associatedObjectName = simGetObjectName(_associatedObjectID);

}

void CameraHandler::handleSimulation(){
	// called when the main script calls: simHandleModule
	if(!_initialized){
		_initialize();
	}

	ros::Time now = ros::Time::now();


	const simFloat currentSimulationTime = simGetSimulationTime();

	if ((currentSimulationTime-_lastPublishedImageTime) >= 1.0/_acquisitionFrequency){

		int resol[2];

		if(simHandleVisionSensor(_associatedObjectID,NULL,NULL)!=-1 &&
				simGetVisionSensorResolution(_associatedObjectID,resol)!=-1){

			// Populate the sensor_msgs::Image message
			sensor_msgs::Image image_msg;
			image_msg.header.stamp = now;

			std::string frame_id=_associatedObjectName;
			if (frame_id[0] != '/'){
				frame_id = "/" + frame_id;
			}

			image_msg.header.frame_id = frame_id;


			//            char* cameraName = new(char[_associatedObjectName.size()]);
			//            memcpy(cameraName,_associatedObjectName.c_str(),_associatedObjectName.size()*sizeof(char));
			//            simReleaseBuffer(cameraName);

			image_msg.width=resol[0]; //Set the width of the image
			image_msg.height=resol[1]; //Set the height of the image
			image_msg.is_bigendian=0;

			if (_cameraIsFloat){
				if(_cameraIsRGB){
					image_msg.encoding=sensor_msgs::image_encodings::TYPE_32FC3;
				} else {
					image_msg.encoding=sensor_msgs::image_encodings::TYPE_32FC1;
				}
			} else {
				if(_cameraIsRGB){
					image_msg.encoding=sensor_msgs::image_encodings::RGB8; //Set the format to be RGB with 8bits per channel
				} else {
					image_msg.encoding=sensor_msgs::image_encodings::MONO8;
				}
			}

			const unsigned int pixPerLine = image_msg.width*sensor_msgs::image_encodings::numChannels(image_msg.encoding);
			image_msg.step=pixPerLine*sensor_msgs::image_encodings::bitDepth(image_msg.encoding)/8; //Set the image stride in bytes
			image_msg.data.resize(image_msg.step*image_msg.height);

			if (_cameraIsRGB){
				if(_cameraIsFloat){
					const simFloat* image_buf = simGetVisionSensorImage(_associatedObjectID);
					simFloat* image_msg_pt = (simFloat*)(image_msg.data.data());
					for(unsigned int i=0; i<image_msg.height; ++i){
						for(unsigned int j=0; j<pixPerLine; ++j){
							image_msg_pt[i*pixPerLine+j] = image_buf[(image_msg.height-i-1)*pixPerLine+j];
						}
					}
					simReleaseBuffer((simChar*)image_buf);
				} else {
					const simUChar* image_buf = simGetVisionSensorCharImage(_associatedObjectID, resol, resol+1);
					simUChar* image_msg_pt = static_cast<simUChar*>(image_msg.data.data());
					for(unsigned int i=0; i<image_msg.height; ++i){
						for(unsigned int j=0; j<pixPerLine; ++j){
							image_msg_pt[i*pixPerLine+j] = image_buf[(image_msg.height-i-1)*pixPerLine+j];
						}
					}
					simReleaseBuffer((simChar*)image_buf);
				}
			} else {
				if (_cameraIsFloat){					
					simFloat* image_msg_pt = (simFloat*)(image_msg.data.data());
#if VREP_VERSION_MAJOR*10000 + VREP_VERSION_MINOR*100 + VREP_VERSION_PATCH < 3*10000 + 3*100 + 1
					const simFloat* image_buf = simGetVisionSensorImage(_associatedObjectID);
					for(unsigned int i=0; i<image_msg.height; ++i){
						for(unsigned int j=0; j<pixPerLine; ++j){
							image_msg_pt[i*pixPerLine+j] = image_buf[3*((image_msg.height-i-1)*pixPerLine+j)]*0.2126f + 
								image_buf[3*((image_msg.height-i-1)*pixPerLine+j)+1]*0.7152f +
								image_buf[3*((image_msg.height-i-1)*pixPerLine+j)+2]*0.0722f;
						}
					}
#else
					const simFloat* image_buf = simGetVisionSensorImage(_associatedObjectID+sim_handleflag_greyscale);
					for(unsigned int i=0; i<image_msg.height; ++i){
						for(unsigned int j=0; j<pixPerLine; ++j){
							image_msg_pt[i*pixPerLine+j] = image_buf[(image_msg.height-i-1)*pixPerLine+j];
						}
					}
#endif
					simReleaseBuffer((simChar*)image_buf);
				} else {
					simUChar* image_msg_pt = static_cast<simUChar*>(image_msg.data.data());
#if VREP_VERSION_MAJOR*10000 + VREP_VERSION_MINOR*100 + VREP_VERSION_PATCH < 3*10000 + 3*100 + 1
					const simUChar* image_buf = simGetVisionSensorCharImage(_associatedObjectID, resol, resol+1);
					for(unsigned int i=0; i<image_msg.height; ++i){
						for(unsigned int j=0; j<pixPerLine; ++j){
							image_msg_pt[i*pixPerLine+j] = (simUChar)((float)image_buf[3*((image_msg.height-i-1)*pixPerLine+j)]*0.2126f + 
								(float)image_buf[3*((image_msg.height-i-1)*pixPerLine+j)+1]*0.7152f +
								(float)image_buf[3*((image_msg.height-i-1)*pixPerLine+j)+2]*0.0722f);
						}
					}
#else
					const simUChar* image_buf = simGetVisionSensorCharImage(_associatedObjectID+sim_handleflag_greyscale, resol, resol+1);
					for(unsigned int i=0; i<image_msg.height; ++i){
						for(unsigned int j=0; j<pixPerLine; ++j){
							image_msg_pt[i*pixPerLine+j] = image_buf[(image_msg.height-i-1)*pixPerLine+j];
						}
					}
#endif
					simReleaseBuffer((simChar*)image_buf);
				}
			}


			_camera_info.header.stamp = now;
			_pubIT.publish(image_msg, _camera_info, now);

			// Publish depth map. Mind that it reuses the same image message
			if(_cameraHasDepth){
				simFloat clipping[2];
				simGetObjectFloatParameter(_associatedObjectID, sim_visionfloatparam_near_clipping, clipping);
				simGetObjectFloatParameter(_associatedObjectID, sim_visionfloatparam_far_clipping, clipping+1);
				sensor_msgs::Image depth_msg;
				depth_msg.header.stamp = now;
				depth_msg.header.frame_id = frame_id;
				depth_msg.width=resol[0]; //Set the width of the image
				depth_msg.height=resol[1]; //Set the height of the image
				depth_msg.is_bigendian=0;
				depth_msg.encoding=sensor_msgs::image_encodings::TYPE_32FC1;
				depth_msg.step=depth_msg.width*sensor_msgs::image_encodings::bitDepth(depth_msg.encoding)/8;
				depth_msg.data.resize(depth_msg.height*depth_msg.step);
				const simFloat* depth_buf = simGetVisionSensorDepthBuffer(_associatedObjectID);
				simFloat* depth_img = reinterpret_cast<simFloat*>(depth_msg.data.data());

				for (unsigned int i = 0; i < depth_msg.height; ++i){
					for (unsigned int j = 0; j < depth_msg.width; ++j){
						depth_img[i*depth_msg.width+j] = clipping[0] + (clipping[1] - clipping[0])*depth_buf[(depth_msg.height-i-1)*depth_msg.width+j];
					}
				}

				simReleaseBuffer((char*)depth_buf);
				_pubDepth.publish(depth_msg);

			}

			_lastPublishedImageTime = currentSimulationTime;
		}
	}

}


void CameraHandler::computeCameraInfo(){
	int resol[2];
	if (simGetVisionSensorResolution(_associatedObjectID,resol)!=-1)
	{
		_camera_info.header.frame_id = "/" + _associatedObjectName;
		_camera_info.width = resol[0];
		_camera_info.height = resol[1];
		simFloat view_angle = M_PI/4;
		const unsigned int viewing_angle_id = 1004;
		simGetObjectFloatParameter(_associatedObjectID, viewing_angle_id, &view_angle);
		double f_x = (_camera_info.width/2.) / tan(view_angle/2.);
		double f_y = f_x;

		_camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
		_camera_info.D.resize(5);
		_camera_info.D[0] = 0;
		_camera_info.D[1] = 0;
		_camera_info.D[2] = 0;
		_camera_info.D[3] = 0;
		_camera_info.D[4] = 0;

		_camera_info.K[0] = f_x; _camera_info.K[1] =   0; _camera_info.K[2] = _camera_info.width/2.0;
		_camera_info.K[3] =   0; _camera_info.K[4] = f_y; _camera_info.K[5] = _camera_info.height/2.0;
		_camera_info.K[6] =   0; _camera_info.K[7] =   0; _camera_info.K[8] = 1;

		_camera_info.R[0] = 1; _camera_info.R[1] = 0; _camera_info.R[2] = 0;
		_camera_info.R[3] = 0; _camera_info.R[4] = 1; _camera_info.R[5] = 0;
		_camera_info.R[6] = 0; _camera_info.R[7] = 0; _camera_info.R[8] = 1;

		_camera_info.P[0] = _camera_info.K[0]; _camera_info.P[1] = 0;         _camera_info.P[2] = _camera_info.K[2]; _camera_info.P[3] = 0;
		_camera_info.P[4] = 0;         _camera_info.P[5] = _camera_info.K[4]; _camera_info.P[6] = _camera_info.K[5]; _camera_info.P[7] = 0;
		_camera_info.P[8] = 0;         _camera_info.P[9] = 0;         _camera_info.P[10] = 1;        _camera_info.P[11] = 0;

	}
}


bool CameraHandler::setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res) {
	//    ROS_INFO("Setting camera parameters for %s.", _associatedObjectName.c_str());

	const int width = req.camera_info.width;
	const int height = req.camera_info.height;

	// check input compatibility
	// TODO: do it better
	if (fabs(req.camera_info.K[0] - req.camera_info.K[4]) > 1e-5 ||
			fabs(req.camera_info.K[2] - width/2.0) > 1e-5 ||
			fabs(req.camera_info.K[5] - height/2.0) > 1e-5) {
		res.success = false;
		res.status_message = std::string("Specified input parameters are not compatible with v-rep.");
		return res.success;
	}


	const simFloat view_angle = 2.0*atan(width/(2.*req.camera_info.K[0]));

	const unsigned int resolution_x_id = 1002;
	const unsigned int resolution_y_id = 1003;
	const unsigned int viewing_angle_id = 1004;

	if (simSetObjectIntParameter(_associatedObjectID, resolution_x_id, width) == 1 &&
			simSetObjectIntParameter(_associatedObjectID, resolution_y_id, height) == 1 &&
			simSetObjectFloatParameter(_associatedObjectID, viewing_angle_id, view_angle) == 1){

		res.success = true;
		res.status_message = std::string("Correctly set camera parameters.");
	} else {
		res.success = false;
		res.status_message = std::string("Error setting camera parameters.");
	}
	computeCameraInfo();
	return res.success;

}


void CameraHandler::_initialize(){
	if (_initialized)
		return;

	// Remove # chars for compatibility with ROS
	std::string objectName(_associatedObjectName);
	std::replace( objectName.begin(), objectName.end(), '#', '_');
	_pubIT = _it.advertiseCamera(objectName, 1);
	_service = _nh.advertiseService(objectName + "/set_camera_info", &CameraHandler::setCameraInfo, this);

	// get some data from the main object
	std::vector<unsigned char> developerCustomData;
	getDeveloperCustomData(developerCustomData);

	// 2. From that retrieved data, try to extract sub-data of interest
	std::vector<unsigned char> tempMainData;

	std::stringstream ss;
	if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::CAMERA_DATA_RGB,tempMainData)){
		_cameraIsRGB=CAccess::pop_int(tempMainData);
		if (_cameraIsRGB){
			ss << "- [" << _associatedObjectName << "] Camera is RGB." << std::endl;
		} else {
			ss << "- [" << _associatedObjectName << "] Camera is grayscale." << std::endl;
		}
	} else {
		_cameraIsRGB = true;
		ss << "- [" << _associatedObjectName << "] Camera color type not specified. Using rgb as default" << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::CAMERA_DATA_HAS_DEPTH,tempMainData)){
		_cameraHasDepth=CAccess::pop_int(tempMainData);
		if (_cameraHasDepth){
			ss << "- [" << _associatedObjectName << "] Camera has a depth sensor." << std::endl;
			std::string objectName(_associatedObjectName);
			std::replace( objectName.begin(), objectName.end(), '#', '_');
			_pubDepth = _it.advertise(objectName + "/depthMap", 1);
		} else {
			ss << "- [" << _associatedObjectName << "] Camera does not have a depth sensor." << std::endl;
		}
	} else {
		_cameraHasDepth = false;
		ss << "- [" << _associatedObjectName << "] Presence of depth sensor not specified. Assuming no depth sensor by default." << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::CAMERA_DATA_USE_FLOAT,tempMainData)){
		_cameraIsFloat=CAccess::pop_int(tempMainData);
		if (_cameraIsFloat){
			ss << "- [" << _associatedObjectName << "] Camera uses float encoding." << std::endl;
		} else {
			ss << "- [" << _associatedObjectName << "] Camera uses char encoding." << std::endl;
		}
	} else {
		_cameraIsFloat = false;
		ss << "- [" << _associatedObjectName << "] Camera encoding not specified. Using char by default." << std::endl;
	}

	if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::CAMERA_DATA_FREQ,tempMainData)){
		_acquisitionFrequency=CAccess::pop_float(tempMainData);
		ss << "- [" << _associatedObjectName << "] Camera acquisition frequency: " << _acquisitionFrequency << "." << std::endl;
	} else {
		_acquisitionFrequency = 30.0;
		ss << "- [" << _associatedObjectName << "] Camera acquisition frequency not specified. Using 30Hz as default." << std::endl;
	}

	// Compute the intrinsic parameters of the camera
	computeCameraInfo();

	ss << "- [" << _associatedObjectName << "] Camera intrinsic matrix K = [";
	for (unsigned i=0; i<3;++i){
		for (unsigned j=0; j<3;++j){
			ss << _camera_info.K[3*i+j] << (j<2 ? ", " : (i<2 ? "; " : "]\n"));
		}
	}

	// Print in the external console
	ConsoleHandler::printInConsole(ss);

	_lastPublishedImageTime = -1e5;
	_initialized=true;
}


bool CameraHandler::endOfSimulation(){
	_pubIT.shutdown();
	_pubDepth.shutdown();
	return GenericObjectHandler::endOfSimulation();
}


PLUGINLIB_EXPORT_CLASS(CameraHandler, GenericObjectHandler)
