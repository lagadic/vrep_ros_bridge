#ifndef QUADROTOR_HANDLER_H
#define QUADROTOR_HANDLER_H

#include "vrep_ros_plugin/GenericObjectHandler.h"

#include "vrep_ros_plugin/access.h"
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

/**
 * Handler of a Quadrotor object.
 */
class QuadrotorHandler : public GenericObjectHandler
{
public:
	QuadrotorHandler();
	~QuadrotorHandler();

	void synchronize();
	void handleSimulation();
	unsigned int getObjectType() const;

protected:

	void _initialize();

	/**
	 * Ratio between force (\f$ f\f$) and torque (\f$\tau\f$) produced by the propeller, i.e. \f$\tau = \alpha f\f$
	 */
	float _torqueToForceRatio;

	/**
	 * Handles of the four propellers.
	 */
	int _handleOfJoint[4];

	/**
	 * Handle of the robot CoM.
	 */
	int _handleOfCoM;

	/**
	 * Handle of the robot CoM.
	 */
	simFloat _quadrotorMass;

	/**
     * Position of the four propellers w.r.t. the robot CoM.
     */
	simFloat _jointPosition[4][3];

	/**
	 * Publisher for the robot status.
	 */
	ros::Publisher _pubPose;
	/**
	 * Publisher for the robot status.
	 */
	ros::Publisher _pubTwist;


	/**
	 * Publisher for the robot status.
	 */
	ros::Publisher _pubIMU;

	/**
	 * Subscriber for force commands.
	 */
	ros::Subscriber _sub;

	/**
	 * Timer for received commands.
	 */
	ros::Time _lastReceivedCmdTime;

    /**
     * Timer for printing messages.
     */
    ros::Time _lastPrintedMsg;

	/**
     * Last received TKCommands message.
     */
	sensor_msgs::JointState _lastReceivedCmd;

	/**
	 * Last received TKMotorCommands commands.
	 */
	std::vector<double> _Commands;

	/**
	 * Time of the last Obj twist.
	 */
	simFloat _lastPublishedStatusTime;

	/**
	 * Obj twist frequency
	 */

	double _ObjStatusFrequency;

	/**
	 * Callback for command message reception.
	 * @param msg Received force command message.
	 */
	void CommandsCallback(const sensor_msgs::JointStateConstPtr& msg);


};


#endif // ndef QUADROTOR_HANDLER_H
