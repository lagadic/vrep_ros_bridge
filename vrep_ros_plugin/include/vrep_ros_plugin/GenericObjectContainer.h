#ifndef GENERIC_OBJECT_CONTAINER_H
#define GENERIC_OBJECT_CONTAINER_H

#include <pluginlib/class_loader.h>

#include <vector>
#include "GenericObjectHandler.h"

#include <map>

#include <rosgraph_msgs/Clock.h>

class GenericObjectContainer
{

public:

    /**
     * Constructor.
     */
	GenericObjectContainer();

	/**
	 * Destroy the container after removing all objects inside.
	 */
	virtual ~GenericObjectContainer();

	/**
	 * Remove all objects from the container.
	 */
	void removeAll();

	/**
	 * Remove the object corresponding to the given ID from the container.
	 * @param theID The unique ID of the object to be removed.
	 */
	void removeFromID(int theID);

	/**
	 * Insert an object in the container.
	 * @param objectHandler A pointer to the object to be inserted.
	 * @return The ID of the inserted object.
	 */
	int insert(boost::shared_ptr<GenericObjectHandler> objectHandler);

	/**
	 * Returns the object corresponding to a given unique ID.
	 * @param theID The unique ID of the object to be returned.
	 * @return A pointer to the object corresponding to the given ID or NULL if the object was not found.
	 */
	boost::shared_ptr<GenericObjectHandler> getFromID(int theID);

	/**
	 * Get the container of an object.
	 * @param theAssociatedObjectID The unique ID of the object.
	 * @return The object element in the container.
	 */
	boost::shared_ptr<GenericObjectHandler> getFromAssociatedObject(int theAssociatedObjectID);

	/**
	 * Returns the object correspImuonding to a container index.
	 * @param ind The container index of the object to be returned.
	 * @return A pointer to the object corresponding to the given index or NULL if the object was not found.
	 */
	boost::shared_ptr<GenericObjectHandler> getFromIndex(int ind);

	/**
	 * Get the number of elements in the container.
	 * @return The number of elements in the container.
	 */
	int getCount();

	/**
	 *   This is called every time the scene content in V-REP might have changed (e.g. object added/removed/etc.).
	 *   The method synchronizes the container with the current V-REP's scene content: we need to make sure that every object in V-REP's scene has exactly one element in this container.
	 */
	void actualizeForSceneContent();

	/**
	 * This is called at the start of simulation.
	 */
	void startOfSimulation();

    /**
     * This is called at the end of simulation.
     */
	void endOfSimulation();

	/**
	 * This is called at each simulation step, i.e. when the main script calls: simHandleModule.
	 */
	void handleSimulation();

    /**
     * A Lua custom command that retrieves all custom data from an object and print it in the format described in \p CAccess class documentation.
     * @param p Lua callback parameter. See V-REP main documentation for details.
     */
    static void simExtGetAllCustomData(SLuaCallBack* p);

    /**
     * A Lua custom command that retrieves the custom data corresponding to a certain header (among those defined in \p CustomDataHeaders) from an object.
     * @param p Lua callback parameter. See V-REP main documentation for details.
     */
    static void simExtGetCustomDataFromHeader(SLuaCallBack* p);

    /**
     * A Lua custom command that sets the floating point custom data corresponding to a certain header (among those defined in \p CustomDataHeaders) from an object.
     * TODO: use binding to pass the object container so that the object can be initialized each time the custom data are modified.
     * @param p Lua callback parameter. See V-REP main documentation for details.
     */
    static void simExtSetFloatCustomDataFromHeader(SLuaCallBack* p);

    /**
     * A Lua custom command that sets the floating point vector custom data corresponding to a certain header (among those defined in \p CustomDataHeaders) from an object.
     * TODO: use binding to pass the object container so that the object can be initialized each time the custom data are modified.
     * @param p Lua callback parameter. See V-REP main documentation for details.
     */
    static void simExtSetFloatArrayCustomDataFromHeader(SLuaCallBack* p);

    /**
     * A Lua custom command that sets the integer point custom data corresponding to a certain header (among those defined in \p CustomDataHeaders) from an object.
     * TODO: use binding to pass the object container so that the object can be initialized each time the custom data are modified.
     * @param p Lua callback parameter. See V-REP main documentation for details.
     */
    static void simExtSetIntCustomDataFromHeader(SLuaCallBack* p);

protected:

	std::vector<boost::shared_ptr<GenericObjectHandler> > _allObjects;

	pluginlib::ClassLoader<GenericObjectHandler> _object_handler_loader;
	std::map<std::string, boost::shared_ptr<GenericObjectHandler> > _allExistingPlugins;

  ros::NodeHandle _nh;
  ros::Publisher _pubClock;
  rosgraph_msgs::Clock _clock_msg;
};

#endif // GENERIC_OBJECT_CONTAINER_H



















