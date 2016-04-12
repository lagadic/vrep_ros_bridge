

#ifndef GETPOSE_HANDLER_H
#define GETPOSE_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>


class GetPoseHandler : public GenericObjectHandler
{
public:
	GetPoseHandler();
	~GetPoseHandler();

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
	 * Publisher for Obj Pose.
	 */
	ros::Publisher _pub;

	/**
	 * Time of the last Obj Pose.
	 */
	simFloat _lastPublishedObjPoseTime;

	/**
	 * Obj Pose frequency
	 */
	double _ObjPoseFrequency;



};


#endif // ndef GETPOSE_HANDLER_H
