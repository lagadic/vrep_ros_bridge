

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

	const static unsigned int DRAWING_DATA_MAIN=800;

	const static unsigned int DRAWING_DATA_WIDTH=DRAWING_DATA_MAIN+1;

	const static unsigned int DRAWING_DATA_DIFFUSE=DRAWING_DATA_WIDTH+1;
	const static unsigned int DRAWING_DATA_SPECULAR=DRAWING_DATA_DIFFUSE+1;
	const static unsigned int DRAWING_DATA_EMISSION=DRAWING_DATA_SPECULAR+1;

	const static unsigned int DRAWING_DATA_MARKERS=DRAWING_DATA_EMISSION+1;
	const static unsigned int DRAWING_DATA_MARKERS_DIFFUSE=DRAWING_DATA_MARKERS+1;
	const static unsigned int DRAWING_DATA_MARKERS_SPECULAR=DRAWING_DATA_MARKERS_DIFFUSE+1;
	const static unsigned int DRAWING_DATA_MARKERS_EMISSION=DRAWING_DATA_MARKERS_SPECULAR+1;

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


	simInt _width; /// Line width
	simFloat _diffuse[3]; /// Line diffuse color
	simFloat _specular[3]; /// Line specular color
	simFloat _emission[3]; /// Line emission color
};
