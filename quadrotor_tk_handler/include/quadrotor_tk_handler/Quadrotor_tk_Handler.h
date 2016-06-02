// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.5 on October 26th 2013

#ifndef QUADROTOR_TK_HANDLER_H
#define QUADROTOR_TK_HANDLER_H

#include "vrep_ros_plugin/GenericObjectHandler.h"

#include "vrep_ros_plugin/access.h"
#include <ros/time.h>
#include <telekyb_msgs/TKMotorCommands.h>
#include <telekyb_msgs/TKCommands.h>

/**
 * Handler of a Quadrotor object for use with Telekyb2.
 * It automatically subscribes to telekyb_msgs::TKMotorCommands messages for receiveng thrust commands to apply to the four propellers.
 * It also publishes the status of the quadrotor using telekyb_msgs::TKState messages.
 */
class Quadrotor_tk_Handler : public GenericObjectHandler
{
public:
	Quadrotor_tk_Handler();
	~Quadrotor_tk_Handler();

	void synchronize();
	void handleSimulation();
	unsigned int getObjectType() const;

protected:

	void _initialize();
	/**
	 * This variable is needed to store the previousTime to perform an Integral of the angles to use an
	 * Integral part into the controller.
	 */
	ros::Time _previousTime;
	/**
	 * The following variables are needed for the Integral part of the PID (roll and pitch respectively)
	 */
	float _integralTermRoll;
	float _integralTermPitch;

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
	ros::Publisher _pub;

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
	 * Last received TKMotorCommands commands.
	 */
	std::vector<double> _tkMotorCommands;

	/**
	 * Last received TKCommands message.
	 */
	telekyb_msgs::TKCommands _tkCommands;

	/**
	 * Internal roll/pitch control cutoff frequency.
	 */
	simFloat _att_cutoff;

	/**
	 * Internal roll/pitch control damping factor.
	 */
	simFloat _att_damping;

	/**
	 * Internal yaw rate control proportional gain.
	 */
	simFloat _kp_yaw;

	/**
	 * Quadrotor control mode. See \ref QuadrotorCtrlMode.
	 */
	CustomDataHeaders::QuadrotorCtrlMode _ctrlMode;

	/**
	 * Callback for force command (TKMotorCommands) message reception.
	 * @param msg Received force command message.
	 */
	void tkMotorCommandsCallback(const telekyb_msgs::TKMotorCommands::ConstPtr& msg);

	/**
	 * Callback for thrust/roll/pitch/yaw command (TKCommands) message reception.
	 * @param msg Received thrust/roll/pitch/yaw command message.
	 */
	void tkCommandsCallback(const telekyb_msgs::TKCommands::ConstPtr& msg);
};


#endif // ndef QUADROTOR_TK_HANDLER_H
