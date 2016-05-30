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

	/// The main identifier of a Set Twist object.
	const static unsigned int SET_OBJ_TWIST_DATA_MAIN=550;
	const static unsigned int SET_OBJ_TWIST_DATA_LIN_GAIN=SET_OBJ_TWIST_DATA_MAIN+1;
	const static unsigned int SET_OBJ_TWIST_DATA_ANG_GAIN=SET_OBJ_TWIST_DATA_LIN_GAIN+1;

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

	simFloat _linGain;
	simFloat _angGain;
};


#endif // ndef SETTWIST_HANDLER_H
