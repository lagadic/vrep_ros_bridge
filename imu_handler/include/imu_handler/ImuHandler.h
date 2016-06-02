

#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>
//#include <telekyb_base/Filter/IIRFilter.hpp>

class ImuHandler : public GenericObjectHandler
{
public:
	ImuHandler();
	~ImuHandler();

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

	/**
     * @copydoc GenericObjectHandler::endOfSimulation()
     */
	bool endOfSimulation();

protected:

    /**
     * @copydoc GenericObjectHandler::_initialize()
     */
	void _initialize();

	/**
	 * Handles of the imu mass.
	 */
	int _handleOfMass;

    /**
     * Imu mass.
     */
    simFloat _mass;

	/**
	 * Handle of the force sensor.
	 */
	int _handleOfForceSensor;

	/**
	 * Publisher for the imu readings.
	 */
	ros::Publisher _pub;
	/**
	 * Imu frequency
	 */
	double _acquisitionFrequency;

	/**
	 * Time of the last published image.
	 */
	simFloat _lastPublishedImu;

	//telekyb::IIRFilter* _forceFilter[3];
	double _forceFilterCutoff;
};


#endif // ndef IMU_HANDLER_H
