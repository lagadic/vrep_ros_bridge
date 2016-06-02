

#ifndef FORCE_SENSOR_HANDLER_H
#define FORCE_SENSOR_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>


class Force_sensorHandler : public GenericObjectHandler
{
public:
	Force_sensorHandler();
	~Force_sensorHandler();

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
	 * Handle of the force sensor.
	 */
	//int _handleOfForceSensor;

	/**
	 * Publisher for the force sensor readings.
	 */
	ros::Publisher _pub;
	/**
	 * Force sensor frequency
	 */
	double _acquisitionFrequency;

	/**
	 * Time of the last published image.
	 */
	simFloat _lastPublishedForceSensor;

};


#endif // ndef FORCE_SENSOR_HANDLER_H
