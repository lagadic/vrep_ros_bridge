

#include "vrep_ros_plugin/GenericObjectHandler.h"
#include "vrep_ros_plugin/access.h"
#include "v_repLib.h"

GenericObjectHandler::GenericObjectHandler():
_id(-1),
_associatedObjectUniqueID(-1),
_associatedObjectID(-1),
_initialized(false),
_nh(ros::this_node::getName()){
}

GenericObjectHandler::~GenericObjectHandler(){
}

void GenericObjectHandler::setID(int newID){
    _id=newID;
}

int GenericObjectHandler::getID(){
    return(_id);
}

void GenericObjectHandler::setAssociatedObject(int objID,int objUniqueID){
    _associatedObjectID=objID;
    _associatedObjectUniqueID=objUniqueID;
}

int GenericObjectHandler::getAssociatedObject(){
    return(_associatedObjectID);
}

int GenericObjectHandler::getAssociatedObjectUniqueId(){
    return(_associatedObjectUniqueID);
}

void GenericObjectHandler::synchronizeSceneObject(){
    // We update GenericObjectHandler's associated scene object custom data:
//    putTagToSceneObject(_associatedObjectID,0.0);
}

void GenericObjectHandler::synchronize(){
}

void GenericObjectHandler::getDeveloperCustomData(std::vector<unsigned char> &developerCustomData){
    const uint buffSize=simGetObjectCustomDataLength(_associatedObjectID,CustomDataHeaders::DEVELOPER_DATA_HEADER);
    developerCustomData.resize(buffSize);
    simGetObjectCustomData(_associatedObjectID,CustomDataHeaders::DEVELOPER_DATA_HEADER,(simChar*)developerCustomData.data());
}


void GenericObjectHandler::startOfSimulation(){
    _initialize();
}

bool GenericObjectHandler::endOfSimulation(){
    _initialized=false;
    return(false); // We don't want this object automatically destroyed at the end of simulation
}
