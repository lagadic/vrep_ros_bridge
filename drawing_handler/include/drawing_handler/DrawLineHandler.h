

#pragma once

#include <vrep_ros_plugin/GenericObjectHandler.h>
#include <geometry_msgs/PolygonStamped.h>

class DrawLineHandler : public GenericObjectHandler
{
public:
	DrawLineHandler();
	~DrawLineHandler();

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

	static const unsigned int DRAWING_DATA_MAIN; ///< Main custom data. Also used to specify drawing frequency.
	static const unsigned int DRAWING_DATA_WIDTH; ///< Custom data used to specify line width.
	static const unsigned int DRAWING_DATA_DIFFUSE; ///< Custom data used to specify line diffuse color component.
	static const unsigned int DRAWING_DATA_SPECULAR; ///< Custom data used to specify line specular color component.
	static const unsigned int DRAWING_DATA_EMISSION; ///< Custom data used to specify line emissive color component.
	static const unsigned int DRAWING_DATA_MARKERS; ///< Custom data used to specify if markers should be drawn. Not implemented yet.
	static const unsigned int DRAWING_DATA_MARKERS_DIFFUSE; ///< Custom data used to specify markers diffuse color component. Not implemented yet.
	static const unsigned int DRAWING_DATA_MARKERS_SPECULAR; ///< Custom data used to specify markers specular color component. Not implemented yet.
	static const unsigned int DRAWING_DATA_MARKERS_EMISSION; ///< Custom data used to specify markers emissive color component. Not implemented yet.

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
	geometry_msgs::PolygonStampedConstPtr _line;

	/**
	 * Callback for velocity commands.
	 */
	void lineCallback(geometry_msgs::PolygonStampedConstPtr msg);

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


	simInt _width; ///< Line width
	simFloat _diffuse[3]; ///< Line diffuse color
	simFloat _specular[3]; ///< Line specular color
	simFloat _emission[3]; ///< Line emission color
};
