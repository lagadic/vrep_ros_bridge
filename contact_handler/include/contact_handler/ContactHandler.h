#ifndef CONTACT_HANDLER_H
#define CONTACT_HANDLER_H

#include <vrep_ros_plugin/GenericObjectHandler.h>


/**
 * The contact handler.
 *
 * It reports the dynamic-engine-reported contacts between all scene objects on a topic called "contacts".
 */
class ContactHandler : public GenericObjectHandler
{
public:
	ContactHandler();
	~ContactHandler();

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

	/**
     * @copydoc GenericObjectHandler::endOfSimulation()
     */
	bool endOfSimulation();

protected:

    /**
     * @copydoc GenericObjectHandler::_initialize()
     */
	void _initialize();

	/**
	 * Publisher for the contact readings.
	 */
	ros::Publisher _pub;

	/**
	 * Publisher frequency
	 */
	double _acquisitionFrequency;

	/**
	 * Time of the last published contact.
	 */
	simFloat _lastPublishedContact;

};

#endif // ifndef CONTACT_HANDLER_H