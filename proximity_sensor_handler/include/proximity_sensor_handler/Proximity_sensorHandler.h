#ifndef PROXIMITY_SENSOR_HANDLER_H
#define PROXIMITY_SENSOR_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>


class Proximity_sensorHandler : public GenericObjectHandler
{
public:
	Proximity_sensorHandler();
	~Proximity_sensorHandler();

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
	 * Handle of the Proximity sensor.
	 */
	//int _handleOfProximitySensor;

	/**
	 * Publisher for the Proximity sensor readings.
	 */
	ros::Publisher _pub;
	/**
	 * Proximity sensor frequency
	 */
	double _acquisitionFrequency;

	/**
	 * Time of the last published image.
	 */
	simFloat _lastPublishedProximitySensor;

};


#endif // ndef Proximity_SENSOR_HANDLER_H
