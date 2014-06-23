

#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include "vrep_ros_plugin/GenericObjectHandler.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/SetCameraInfo.h>


class CameraHandler : public GenericObjectHandler
{
public:
	CameraHandler();
	~CameraHandler();

    /**
     * @copydoc GenericObjectHandler::synchronize()
     */
	void synchronize();

	/**
     * @copydoc GenericObjectHandler::handleSimulation()
	 */
	void handleSimulation();

	/**
     * @copydoc GenericObjectHandler::getObjectType()
     */
	unsigned int getObjectType() const;


protected:

    /**
     * @copydoc GenericObjectHandler::_initialize()
     */
	void _initialize();

	/**
	 * 	We create an ImageTransport instance.
	 */
	image_transport::ImageTransport _it;

	/**
	 * Publisher for camera images.
	 */
	image_transport::CameraPublisher _pubIT;

	/**
	 *  Preconstructed msg containing camera information.
	 */
	sensor_msgs::CameraInfo _camera_info;

	/**
	 * Helping method to compute camera information message.
	 */
	void computeCameraInfo();

	/**
	 * Time of the last published image.
	 */
	simFloat _lastPublishedImageTime;

	/**
	 * Camera frequency
	 */
	double _acquisitionFrequency;
	/**
	 * Camera is RGB or not
	 */
	bool _cameraIsRGB;

	/**
	 * Ros service server for setting camera parameters from ros.
	 */
	ros::ServiceServer _service;

    /**
     * Service handler for setting camera parameters from ros.
     * @param req The desired camera parameters.
     * @param res Return 0 if the parameters have been set properly, 1 otherwise (some more information may be contained in the attached string)
     * @return
     */
    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
};


#endif // ndef CAMERA_HANDLER_H
