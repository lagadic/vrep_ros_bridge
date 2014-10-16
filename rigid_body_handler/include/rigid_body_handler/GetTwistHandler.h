

#ifndef GETTWIST_HANDLER_H
#define GETTWIST_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>


class GetTwistHandler : public GenericObjectHandler
{
public:
	GetTwistHandler();
	~GetTwistHandler();

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
	 * Publisher for Obj twist.
	 */
	ros::Publisher _pub;

	/**
	 * Time of the last Obj twist.
	 */
	simFloat _lastPublishedObjTwistTime;

	/**
	 * Obj twist frequency
	 */
	double _ObjTwistFrequency;

	/**
	 * Specifies if the object is static or dynamically enabled
	 */
	simInt _isStatic;

};


#endif // ndef GETTWIST_HANDLER_H
