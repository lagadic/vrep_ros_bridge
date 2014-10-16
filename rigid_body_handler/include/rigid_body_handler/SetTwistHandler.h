#ifndef SETTWIST_HANDLER_H
#define SETTWIST_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <geometry_msgs/TwistStamped.h>

/**
 * Handler to control the velocity of an object.
 * It automatically subscribes to geometry_msgs/TwistStamped messages for receiving twist commands.
 * To move the object, generate a message geometry_msgs/TwistStamped in the topic /vrep/object_name_Vrep/SetTwist
 */



class SetTwistHandler : public GenericObjectHandler
{
public:
	SetTwistHandler();
	~SetTwistHandler();

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
	 * Subscriber for Obj twist.
	 */
	ros::Subscriber _sub;

	/**
     * Last received TwistCommand message.
     */
	geometry_msgs::TwistStamped _twistCommands;

	/**
     * Callback for velocity commands.
     */
	void TwistCommandCallback(const geometry_msgs::TwistStamped& msg);

	/**
	 * Specifies if the object is static or dynamically enabled
	 */
	simInt _isStatic;
};


#endif // ndef SETTWIST_HANDLER_H
