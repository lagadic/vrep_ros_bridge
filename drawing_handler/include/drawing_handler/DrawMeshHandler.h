

#pragma once

#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <shape_msgs/Mesh.h>


class DrawMeshHandler : public GenericObjectHandler
{
public:
	DrawMeshHandler();
	~DrawMeshHandler();

	/**
	 * @copydoc GenericObjectHandler::synchronize()
	 */
	void synchronize();

	/**
	 * @copydoc GenericObjectHandler::_initialize()
	 */
	bool endOfSimulation();

	/**
	 * @copydoc GenericObjectHandler::handleSimulation()
	 */
	void handleSimulation();

	/**
	 * @copydoc GenericObjectHandler::getObjectType()
	 */
	unsigned int getObjectType() const;

	static const unsigned int DATA_MAIN; ///< Main custom data. Also used to specify drawing frequency.
	static const unsigned int DATA_WIDTH; ///< Custom data used to specify line width.
	static const unsigned int DATA_DIFFUSE; ///< Custom data used to specify surface diffuse color component.
	static const unsigned int DATA_SPECULAR; ///< Custom data used to specify surface specular color component.
	static const unsigned int DATA_EMISSION; ///< Custom data used to specify surface emissive color component.
	static const unsigned int DATA_TRANSPARENCY; ///< Custom data used to specify surface transparency.

protected:

	/**
	 * Register custom lua variables for use in Lua scripts
	 */
	void registerCustomVariables() const;

	/**
	 * @copydoc GenericObjectHandler::_initialize()
	 */
	void _initialize();

	/**
	 * Subscriber for receiving line points.
	 */
	ros::Subscriber _sub;

	/**
	 * Last received message.
	 */
	shape_msgs::MeshConstPtr _lastMsg;

	/**
	 * Callback for velocity commands.
	 */
	void msgCallback(shape_msgs::MeshConstPtr msg);

	/**
	 * Simulation time of the last drawing.
	 */
	simFloat _lastTime;

	/**
	 * Drawing frequency
	 */
	simFloat _frequency;

	/**
	 * Drawing object handle
	 */
	simInt _drawingObject;


	simInt _width; /// Mesh line width
	simFloat _diffuse[3]; /// Mesh diffuse color
	simFloat _specular[3]; /// Mesh specular color
	simFloat _emission[3]; /// Mesh emission color
	simInt _transparency; /// Mesh transparency level: 0=100%, 1=50%, 2=25%, 3=12.5%
};
