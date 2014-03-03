
#include <pluginlib/class_list_macros.h>
#include <camera_handler/CameraHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <Eigen/Geometry>

#include <vrep_ros_plugin/ConsoleHandler.h>

CameraHandler::CameraHandler() : GenericObjectHandler(),
    _acquisitionFrequency(30.0),
    _lastPublishedImageTime(0.0),
    _cameraIsRGB(true),
    _it(_nh){
}

CameraHandler::~CameraHandler(){
}

unsigned int CameraHandler::getObjectType() const {
    return CustomDataHeaders::CAMERA_DATA_MAIN;
}

void CameraHandler::synchronize(){

    _associatedObjectName = simGetObjectName(_associatedObjectID);

    // Remove # chars for compatibility with ROS
    std::string objectName(_associatedObjectName);
    std::replace( objectName.begin(), objectName.end(), '#', '_');
    _pubIT = _it.advertiseCamera(objectName, 1);

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


            char* cameraName = new(char[_associatedObjectName.size()]);
            memcpy(cameraName,_associatedObjectName.c_str(),_associatedObjectName.size()*sizeof(char));
            simReleaseBuffer(cameraName);

            image_msg.height=resol[1]; //Set the height of the image
            image_msg.width=resol[0]; //Set the width of the image
            image_msg.is_bigendian=0;

            float* image_buf = simGetVisionSensorImage(_associatedObjectID);

            if (_cameraIsRGB){
                image_msg.encoding=sensor_msgs::image_encodings::RGB8; //Set the format to be RGB with 8bits per channel
                image_msg.step=image_msg.width*3; //Set the image stride in bytes

                const int data_len=image_msg.step*image_msg.height;
                image_msg.data.resize(data_len);

                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                    image(image_buf,image_msg.height,image_msg.step);
                Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                   imageMsg(image_msg.data.data(),image_msg.height,image_msg.step);
//                Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::OuterStride<> >
//                    imageMsg(image_buf+(resol[1]-1)*3*resol[0],image_msg.height,image_msg.step, Eigen::OuterStride<>(-3*(int)(resol[0])) );

//                imageMsg = (255.1f*image).cast<unsigned char>();
//                for (uint i = 0; i<image_msg.height; ++i){
//                    imageMsg.row(image_msg.height-i-1) = (255.1f*image.row(i)).cast<unsigned char>();
//                }
                imageMsg = (255.1f*image.colwise().reverse()).cast<unsigned char>();


            } else {
                image_msg.encoding=sensor_msgs::image_encodings::MONO8;
                image_msg.step=image_msg.width; //Set the image stride in bytes

                const int data_len=image_msg.step*image_msg.height;
                image_msg.data.resize(data_len);

//                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride< Eigen::Dynamic, 3> >
//                    imageR(image_buf+(resol[1]-1)*3*resol[0],image_msg.height,image_msg.step, Eigen::Stride<Eigen::Dynamic, 3>(-3*(int)(resol[0]), 3) );
//                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride< Eigen::Dynamic, 3> >
//                    imageG(image_buf+(resol[1]-1)*3*resol[0]+1,image_msg.height,image_msg.step, Eigen::Stride<Eigen::Dynamic, 3>(-3*(int)(resol[0]), 3) );
//                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride< Eigen::Dynamic, 3> >
//                    imageB(image_buf+(resol[1]-1)*3*resol[0]+2,image_msg.height,image_msg.step, Eigen::Stride<Eigen::Dynamic, 3>(-3*(int)(resol[0]), 3) );

                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride< Eigen::Dynamic, 3> >
                    imageR(image_buf,image_msg.height,image_msg.step, Eigen::Stride<Eigen::Dynamic, 3>(3*(int)(resol[0]), 3) );
                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride< Eigen::Dynamic, 3> >
                    imageG(image_buf+1,image_msg.height,image_msg.step, Eigen::Stride<Eigen::Dynamic, 3>(3*(int)(resol[0]), 3) );
                Eigen::Map< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, 0, Eigen::Stride< Eigen::Dynamic, 3> >
                    imageB(image_buf+2,image_msg.height,image_msg.step, Eigen::Stride<Eigen::Dynamic, 3>(3*(int)(resol[0]), 3) );

                Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >
                   imageMsg(image_msg.data.data(),image_msg.height,image_msg.step);

//                const Eigen::Vector3f coeffs(255.1f/3.0,255.1f/3.0,255.1f/3.0); //RGB to grayscale averaging
                const Eigen::Vector3f coeffs(255.1*0.2126,255.1*0.7152,255.1*0.0722); //RGB to grayscale luminance

//              imageMsg = (coeffs[0]*imageR+coeffs[1]*imageG+coeffs[2]*imageB).cast<unsigned char>();
                // Transform RGB to monochrome and flip rows
//                for (uint i = 0; i<image_msg.height; ++i){
//                    imageMsg.row(image_msg.height-i-1) = (coeffs[0]*imageR.row(i)
//                            +coeffs[1]*imageG.row(i)
//                            +coeffs[2]*imageB.row(i)).cast<unsigned char>();
//                }

                imageMsg = ((coeffs[0]*imageR+coeffs[1]*imageG+coeffs[2]*imageB).colwise().reverse()).cast<unsigned char>();

            }

            simReleaseBuffer((char*)image_buf);
            _camera_info.header.stamp = now;
            _pubIT.publish(image_msg, _camera_info, now);
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

        _camera_info.K[0] = f_x; _camera_info.K[1] =   0; _camera_info.K[2] = _camera_info.width/2;
        _camera_info.K[3] =   0; _camera_info.K[4] = f_y; _camera_info.K[5] = _camera_info.height/2;
        _camera_info.K[6] =   0; _camera_info.K[7] =   0; _camera_info.K[8] = 1;

        _camera_info.R[0] = 1; _camera_info.R[1] = 0; _camera_info.R[2] = 0;
        _camera_info.R[3] = 0; _camera_info.R[4] = 1; _camera_info.R[5] = 0;
        _camera_info.R[6] = 0; _camera_info.R[7] = 0; _camera_info.R[8] = 1;

        _camera_info.P[0] = _camera_info.K[0]; _camera_info.P[1] = 0;         _camera_info.P[2] = _camera_info.K[2]; _camera_info.P[3] = 0;
        _camera_info.P[4] = 0;         _camera_info.P[5] = _camera_info.K[4]; _camera_info.P[6] = _camera_info.K[5]; _camera_info.P[7] = 0;
        _camera_info.P[8] = 0;         _camera_info.P[9] = 0;         _camera_info.P[10] = 1;        _camera_info.P[11] = 0;

    }
}


void CameraHandler::_initialize(){
    if (_initialized)
        return;

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

    if (CAccess::extractSerializationData(developerCustomData,CustomDataHeaders::CAMERA_DATA_FREQ,tempMainData)){
        _acquisitionFrequency=CAccess::pop_float(tempMainData);
        ss << "- [" << _associatedObjectName << "] Camera acquisition frequency: " << _acquisitionFrequency << "." << std::endl;
    } else {
        _acquisitionFrequency = 30.0;
        ss << "- [" << _associatedObjectName << "] Camera acquisition frequency not specified. Using 30Hz as default." << std::endl;
    }

    // Compute the intrinsic parameters of the camera
    computeCameraInfo();

    Eigen::Map<Eigen::Matrix3d> K(&_camera_info.K[0]);
    ss << "- [" << _associatedObjectName << "] Camera intrinsic matrix K = " << std::endl << K <<  std::endl;

    // Print in the external console
    ConsoleHandler::printInConsole(ss);

    _lastPublishedImageTime = -1e5;
    _initialized=true;
}


PLUGINLIB_EXPORT_CLASS(CameraHandler, GenericObjectHandler)
