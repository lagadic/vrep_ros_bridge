

#ifndef MANIPULATOR_HANDLER_H
#define MANIPULATOR_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <vrep_ros_plugin/access.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

class ManipulatorHandler : public GenericObjectHandler
{
public:
	ManipulatorHandler();
	~ManipulatorHandler();

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

	/// Handle of the manipulator joints.
	std::vector<int> _handleOfJoints;

    /// Names of the manipulator joints.
    std::vector<std::string> _jointNames;

	/// Publisher for the joint status.
	ros::Publisher _pub;

	/// Subscriber for the joint commands.
    ros::Subscriber _sub;

	/// Subscriber for the Mobile robots velocity commands .
    ros::Subscriber _subVelMob;

	/// Joint status publication frequency.
	double _acquisitionFrequency;

	/// Time of the last published joint status.
	simFloat _lastPublishedStatus;

    /// Time of the last received joint command.
	simFloat _lastReceivedCmdTime;

    /// Time of the last message printed on the console.
    ros::Time _lastPrintedMsg;

    /// The last received joint command.
    sensor_msgs::JointState _lastReceivedCmd;

    /// Manipulator control mode. See \ref ManipulatorCtrlMode.
    std::vector<CustomDataHeaders::ManipulatorCtrlMode> _jointCtrlMode;

	/// Callback for joint commands.
	void jointCommandCallback(const sensor_msgs::JointStateConstPtr& msg);

	/// Callback for joint commands.
	void VelMobCommandCallback(const geometry_msgs::TwistConstPtr& msg);

	/// Number of found joint
	unsigned int _numJoints;
	/// Number of found joint
	//float _tempjoint;

	/// Lenght mobile robot
	double _axle_lenght;

	// Wheel radius
	double _mb_radius;

	/// Default mode controller
	CustomDataHeaders::ManipulatorCtrlMode _defaultModeCtrl;

};


#endif // ndef MANIPULATOR_HANDLER_H
