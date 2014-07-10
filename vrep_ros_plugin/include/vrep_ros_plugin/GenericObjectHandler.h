#ifndef GENERIC_OBJECT_HANDLER_H
#define GENERIC_OBJECT_HANDLER_H

#include <vector>

#include <ros/ros.h>
#include <v_repTypes.h>

/**
 * Handler of a generic V-REP scene object. This should never be used directly but always inherited by specific handlers.
 */
class GenericObjectHandler
{
public:
	GenericObjectHandler();
	virtual ~GenericObjectHandler();

	/**
	 * Set the ID of this GenericObjectHandler.
	 * @param newID @copydoc GenericObjectHandler::_id
	 */
	void setID(int newID);

	/**
     * Get the ID of this GenericObjectHandler.
     * @return @copydoc GenericObjectHandler::_id
     */
	int getID();

	/**
	 * Set the object associated with this GenericObjectHandler.
	 * @param objID @copydoc GenericObjectHandler::_associatedObjectID
	 * @param objUniqueID @copydoc GenericObjectHandler::_associatedObjectUniqueID
	 */
	void setAssociatedObject(int objID,int objUniqueID);

	/**
	 * Get the scene object associated to this GenericObjectHandler.
	 * @return The handle of the scene object associated to this GenericObjectHandler.
	 */
	int getAssociatedObject();

    /**
     * Get the unique ID of scene object associated to this GenericObjectHandler.
     * @return The unique ID of the scene object associated to this GenericObjectHandler.
     */
	int getAssociatedObjectUniqueId();

	/**
	 * Synchronizes the GenericObjectHandler with the scene. This method is called each time this latter changes.
	 */
	void synchronizeSceneObject();

	/**
	 * Update GenericObjectHandler's data from its associated scene object custom data.
	 */
	virtual void synchronize();

	/**
	 * Get all the developer data attached to the associated scene object (this is custom data added by the developer).
	 */
	void getDeveloperCustomData(std::vector<unsigned char> &developerCustomData);

	/**
	 * This method is called (indirectly) from a Lua-script at the beginning of the simulation.
	 */
	void startOfSimulation();

	/**
     * This method is called (indirectly) from a Lua-script at the end of the simulation.
     */
	virtual bool endOfSimulation();

	/**
     * This method is called (indirectly) from a Lua-script at each simulation step.
     */
	virtual void handleSimulation() = 0;

    /**
     * Get the specific object type for derived classes (i.e. XXXX_DATA_MAIN).
     */
	virtual unsigned int getObjectType() const = 0;

protected:
	
	/**
	 * This method is called to initialize the object handler.
	 * In particular this is also called from GenericObjectHandler::startOfSimulation() at the beginning of the simulation.
	 */
	virtual void _initialize() = 0;

	/**
	 * Unique ID to identify individual GenericObjectHandlers within the container.
	 */
	int _id;

	/**
	 * The unique ID of the scene object associated with this GenericObjectHandler.
	 */
	int _associatedObjectUniqueID;

	/**
	 * The handle of the scene object associated with this GenericObjectHandler.
	 */
	int _associatedObjectID;

	/**
	 * The unique name of the scene object associated with this GenericObjectHandler.
	 */
	std::string _associatedObjectName;

	/**
	 * Set to true when the GenericObjectHandler is initialized.
	 */
    bool _initialized;

    /**
     * Handle of the ros node.
     */
    ros::NodeHandle _nh;

};


#endif // ndef GENERIC_OBJECT_HANDLER_H
