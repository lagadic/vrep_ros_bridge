#include <pluginlib/class_list_macros.h>

#include <contact_handler/ContactHandler.h>
#include <v_repLib.h>
#include <vrep_ros_plugin/access.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ContactsState.h>

#include <vrep_ros_plugin/ConsoleHandler.h>


ContactHandler::ContactHandler() : GenericObjectHandler(),
    _lastPublishedContact(0.0),
    _acquisitionFrequency(-1.0)
  {
}

ContactHandler::~ContactHandler(){
}

unsigned int ContactHandler::getObjectType() const {
    return CustomDataHeaders::CONTACT_DATA_MAIN;
}

void ContactHandler::synchronize(){
    _associatedObjectName = simGetObjectName(_associatedObjectID);
}

void ContactHandler::handleSimulation(){
    // called when the main script calls: simHandleModule
    if(!_initialized){
        _initialize();
    }

    const simFloat currentSimulationTime = simGetSimulationTime();

    if ( ((currentSimulationTime - _lastPublishedContact) >= 1.0/_acquisitionFrequency) ){

        gazebo_msgs::ContactsState contactsMessage;

    	simInt contactHandles[2];
    	simFloat contactInfo[6];

        // simGetContactInfo is asked for contacts by their indices; if there is no contact with the given index,
        // the function returns 0 and we know that we have already iterated through all contacts
        int contact_index = 0;
    	while (0 < simGetContactInfo(sim_handle_all, sim_handle_all, contact_index, contactHandles, contactInfo)) {

			// Construct the contact msg

            gazebo_msgs::ContactState contact;

            contact.collision1_name = std::string(simGetObjectName(contactHandles[0]));
            contact.collision2_name = std::string(simGetObjectName(contactHandles[1]));

            geometry_msgs::Vector3 contactPosition;
            contactPosition.x = contactInfo[0]; contactPosition.y = contactInfo[1]; contactPosition.z = contactInfo[2];
            contact.contact_positions.push_back(contactPosition);

            geometry_msgs::Vector3 force;
            force.x = contactInfo[3];
            force.y = contactInfo[4];
            force.z = contactInfo[5];
            contact.contact_normals.push_back(force);

			geometry_msgs::Wrench wrench;
			wrench.force = force;
            // wrench.torque = ???; // there is no torque info AFAIK
			contact.wrenches.push_back(wrench);
            contact.total_wrench = wrench;

            contactsMessage.states.push_back(contact);

            ++contact_index;
		}

        // Publish the contact message in ROS
        _pub.publish(contactsMessage);
        _lastPublishedContact = currentSimulationTime;
    }
}

void ContactHandler::_initialize(){
    if (_initialized)
        return;

    // start the ROS publisher
    _pub = _nh.advertise<gazebo_msgs::ContactsState>("contacts", 1000);

    // get some data from the main object
    std::vector<unsigned char> developerCustomData;
    getDeveloperCustomData(developerCustomData);

    // 2. From that retrieved data, try to extract sub-data with the CONTACT_DATA_MAIN tag:
    std::vector<unsigned char> tempMainData;
    std::stringstream ss;

    if (CAccess::extractSerializationData(developerCustomData, CustomDataHeaders::CONTACT_DATA_MAIN,tempMainData)){
        _acquisitionFrequency=CAccess::pop_float(tempMainData);
        if (_acquisitionFrequency > 0.0){
            ss << "- ContactHandler [" << _associatedObjectName << "] Frequency publisher: " << _acquisitionFrequency << "." << std::endl;
        } else {
            ss << "- ContactHandler [" << _associatedObjectName << "] Frequency publisher: 50 Hz."  << std::endl;
            _acquisitionFrequency = 50.0;
        }
    } else {
        ss << "- ContactHandler [" << _associatedObjectName << "] Contact frequency publisher not specified. Using 50 Hz as default."  << std::endl;
        _acquisitionFrequency = 50.0;
    }

    ConsoleHandler::printInConsole(ss);

    _lastPublishedContact = -1.0f;
    _initialized=true;
}

bool ContactHandler::endOfSimulation(){

    _initialized=false;
    _pub.shutdown();

    return false;
}

PLUGINLIB_EXPORT_CLASS(ContactHandler, GenericObjectHandler)
