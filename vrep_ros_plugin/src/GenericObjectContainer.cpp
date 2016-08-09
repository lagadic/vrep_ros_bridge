
#include "vrep_ros_plugin/access.h"
#include "v_repLib.h"

#include <vrep_ros_plugin/GenericObjectContainer.h>

#include "vrep_ros_plugin/ConsoleHandler.h"


GenericObjectContainer::GenericObjectContainer():
	_object_handler_loader("vrep_ros_plugin", "GenericObjectHandler"),
  _nh(ros::this_node::getName()){

    std::vector< std::string > plugins = _object_handler_loader.getDeclaredClasses();

    std::stringstream ss;
    ss << "Available ObjectHandlers: " << std::endl;
    for (uint i=0; i<plugins.size();++i){
        ss << plugins[i] << std::endl;
        try{
            _allExistingPlugins.insert ( std::pair<std::string, boost::shared_ptr<GenericObjectHandler> >(plugins[i],_object_handler_loader.createInstance(plugins[i])) );
        } catch(pluginlib::PluginlibException& ex){
          //handle the class failing to load
          ROS_ERROR("The plugin %s failed to load for some reason. Error: %s", plugins[i].c_str(), ex.what());
        }
    }

    _pubClock = _nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    ConsoleHandler::printInConsole(ss);
}

GenericObjectContainer::~GenericObjectContainer(){
	removeAll();
}

void GenericObjectContainer::removeAll(){
	_allObjects.clear();
	_allExistingPlugins.clear();
}

void GenericObjectContainer::removeFromID(int theID){
	for (int i=0;i<getCount();i++)
	{
		if (_allObjects[i]->getID()==theID)
		{
			_allObjects.erase(_allObjects.begin()+i);
			break;
		}
	}
}

int GenericObjectContainer::insert(boost::shared_ptr<GenericObjectHandler> objectHandler){
	int newID=0;
	while (getFromID(newID)!=NULL)
		newID++;
	_allObjects.push_back(objectHandler);
	objectHandler->setID(newID);
	return(newID);
}

boost::shared_ptr<GenericObjectHandler> GenericObjectContainer::getFromID(int theID){
	for (int i=0;i<getCount();i++)
	{
		if (_allObjects[i]->getID()==theID)
			return(_allObjects[i]);
	}
	return boost::shared_ptr<GenericObjectHandler>();
}

boost::shared_ptr<GenericObjectHandler> GenericObjectContainer::getFromIndex(int ind){
	if ( (ind<0)||(ind>=getCount()) )
		return boost::shared_ptr<GenericObjectHandler>();
	return(_allObjects[ind]);
}

boost::shared_ptr<GenericObjectHandler> GenericObjectContainer::getFromAssociatedObject(int theAssociatedObjectID){
	for (int i=0;i<getCount();i++)
	{
		if (_allObjects[i]->getAssociatedObject()==theAssociatedObjectID)
			return(_allObjects[i]);
	}
	return boost::shared_ptr<GenericObjectHandler>();
}

int GenericObjectContainer::getCount(){
	return(int(_allObjects.size()));
}

void GenericObjectContainer::startOfSimulation(){ // Relay the message to all GenericObjectHandler objects:
	for (int i=0;i<getCount();i++)
		_allObjects[i]->startOfSimulation();
}

void GenericObjectContainer::endOfSimulation(){ // Relay the message to all GenericObjectHandler objects:
	for (int i=0;i<getCount();i++)
	{
		if (_allObjects[i]->endOfSimulation())
		{ // Means that this object wants to be destroyed
			_allObjects.erase(_allObjects.begin()+i);
			i--;
		}
	}
}

void GenericObjectContainer::handleSimulation(){
    // Relay the message to all GenericObjectHandler objects:
	for (int i=0;i<getCount();i++)
		_allObjects[i]->handleSimulation();

  _clock_msg.clock = ros::Time(simGetSimulationTime());
  _pubClock.publish(_clock_msg);
}

void GenericObjectContainer::actualizeForSceneContent(){
	// 1. We first check which GenericObjectHandler is not valid anymore, and remove it
	int i=0;
	while (i<int(_allObjects.size()))
	{
	    boost::shared_ptr<GenericObjectHandler> objectHandler=_allObjects[i];
		int associatedObject=objectHandler->getAssociatedObject();
		int uniqueID;
		if (simGetObjectUniqueIdentifier(associatedObject,&uniqueID)==-1)
		{ // the object doesn't exist anymore!
			removeFromID(objectHandler->getID());
			i=-1; // ordering in _allObjects might have changed, we start from the beginning
		}
		else
		{ // An object with that handle exists, is its unique ID same as getAssociatedObjectUniqueId()?
			if (uniqueID!=objectHandler->getAssociatedObjectUniqueId())
			{ // the object doesn't exist anymore! (the object was destroyed and then another object was assigned the same handle (e.g. new scene was loaded))
				removeFromID(objectHandler->getID());
				i=-1; // ordering in _allObjects might have changed, we start from the beginning
			} else 	{
			    // the object still exists. Does it still have its QUADROTOR_DATA_MAIN tag?
				// a. Get all the developer data attached to this object (this is custom data added by the developer):
				int buffSize=simGetObjectCustomDataLength(associatedObject,CustomDataHeaders::DEVELOPER_DATA_HEADER);
				char* datBuff=new char[buffSize];
				simGetObjectCustomData(associatedObject,CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
				std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
				delete[] datBuff;
				// b. From that retrieved data, try to extract sub-data with the QUADROTOR_DATA_MAIN tag:
				std::vector<unsigned char> tempMainData;
				if (!CAccess::extractSerializationData(developerCustomData,objectHandler->getObjectType(),tempMainData)) // the tag is composed by a string and an integer
				{ // No, there is no data with the QUADROTOR_DATA_MAIN tag present! We remove this object:
					removeFromID(objectHandler->getID());
					i=-1; // ordering in _allObjects might have changed, we start from the beginning
				}
			}
		}
		i++;
	}

	// 2. Now all GenericObjectHandler objects in this container are up-to-date. We now go through all objects in the scene
	// that have the QUADROTOR_DATA_MAIN tag, and check if they have a related GenericObjectHandler object in this container.
	// If not, we have to add a related GenericObjectHandler object:

	int objHandle=0;
	int ind=0;
	while (objHandle!=-1)
	{
		objHandle=simGetObjects(ind++,sim_handle_all);
		if (objHandle!=-1)
		{ // Scene object present at index ind
			// Is that scene object already associated with a GenericObjectHandler object?
			if (getFromAssociatedObject(objHandle)==NULL)
			{ // No, not yet
				// Does the object have a QUADROTOR_DATA_MAIN tag?
				// a. Get all the developer data attached to this object (this is custom data added by the developer):
				int buffSize=simGetObjectCustomDataLength(objHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER);
				char* datBuff=new char[buffSize];
				simGetObjectCustomData(objHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER,datBuff);
				std::vector<unsigned char> developerCustomData(datBuff,datBuff+buffSize);
				delete[] datBuff;
				// b. From that retrieved data, try to extract sub-data with the QUADROTOR_DATA_MAIN tag:
				std::vector<unsigned char> tempMainData;
				bool objectFound = false;

				boost::shared_ptr<GenericObjectHandler> objectHandler;

				std::stringstream ss;

				for (std::map< std::string, boost::shared_ptr<GenericObjectHandler> >::iterator it = _allExistingPlugins.begin(); it!=_allExistingPlugins.end(); ++it){
			        if ((objectFound = CAccess::extractSerializationData(developerCustomData, it->second->getObjectType(), tempMainData)) == true) {
                        // Yes, the tag is there. We have to add a new GenericObjectHandler object associated with this scene object:
                        try {
                            objectHandler = _object_handler_loader.createInstance(it->first);
                            insert(objectHandler);
                            int uniqueID;
                            simGetObjectUniqueIdentifier(objHandle,&uniqueID);
                            objectHandler->setAssociatedObject(objHandle,uniqueID);
                            objectHandler->synchronize();
                            ss << "Instantiating new " << it->first << " for "  << simGetObjectName(objHandle) << "." << std::endl;
                            ConsoleHandler::printInConsole(ss);
                        } catch(pluginlib::PluginlibException& ex){
                          //handle the class failing to load
                          ROS_ERROR("The plugin %s failed to load for some reason. Error: %s", it->first.c_str(), ex.what());
                        }
                    }
			    }
			}
		}
	}		
}




void GenericObjectContainer::simExtGetAllCustomData(SLuaCallBack* p){
    if (p->inputArgCount!=1){
        simSetLastError(pluginName,"This function requires 1 input argument.");
        return;
    }

    if (p->outputArgCount>1){
        simSetLastError(pluginName,"This function accepts 1 output argument.");
        return;
    }

    if(p->inputArgTypeAndSize[0]!=sim_lua_arg_int || p->inputInt[0]<0){
        simSetLastError(pluginName,"Wrong input argument type.");
        return;
    }

    if (p->outputArgCount==1 &&
            p->outputArgTypeAndSize[0]!=sim_lua_arg_string){
        simSetLastError(pluginName,"Wrong output argument type.");
        return;
    }

    // if inputs are correct go ahead
    const int objectHandle = p->inputInt[0];


    const uint buffSize=simGetObjectCustomDataLength(objectHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);

    if (buffSize<0){
        simSetLastError(pluginName,"The object does not contain any custom data for this plugin.");
        return;
    }

    std::vector<unsigned char> developerCustomData(buffSize);
    simGetObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,(simChar*)developerCustomData.data());

    std::stringstream ss;
    ss << "[";
    while (developerCustomData.size()>0){
        const int* dataHeader = ((int *)developerCustomData.data());
        const int* dataLength = ((int *)developerCustomData.data()+1);
        const unsigned char * data = developerCustomData.data()+2*sizeof(int);

        ss << std::dec << *dataHeader << ", " << *dataLength << ", ";
        if (*dataHeader == CustomDataHeaders::QUADROTOR_DATA_CTRL_MODE){
            ss << *((int *)data);
        } else if (*dataHeader == CustomDataHeaders::QUADROTOR_DATA_TF_RATIO){
            ss << *((float*)data);
        } else {
            ss << "0x";
            for (uint i=0; i<*dataLength; ++i){
                ss << std::hex << std::setw(2) << std::setfill('0') << (int)data[i];
            }
        }

        developerCustomData.erase(developerCustomData.begin(), developerCustomData.begin()+2*sizeof(int)+*dataLength);
        if (developerCustomData.size()>0){
            ss << "; ";
        } else {
            ss << "]";
        }
    }

    p->outputArgCount=1; // 1 return value (function succeeded)
    p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt));
    p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_string;   // The first return value is an int
    p->outputArgTypeAndSize[2*0+1]=sim_lua_arg_nil;     // Not used (table size if the return value was a table)
    p->outputChar=(simChar*)simCreateBuffer(ss.str().size()+1);
    strcpy(p->outputChar,ss.str().c_str());
}


void GenericObjectContainer::simExtGetCustomDataFromHeader(SLuaCallBack* p){
    if (p->inputArgCount!=2){
        simSetLastError(pluginName,"This function requires 2 input arguments.");
        return;
    }

    if (p->outputArgCount>1){
        simSetLastError(pluginName,"This function accepts 1 output argument.");
        return;
    }

    if(p->inputArgTypeAndSize[0]!=sim_lua_arg_int || p->inputInt[0]<0 ||
            p->inputArgTypeAndSize[2]!=sim_lua_arg_int){
        simSetLastError(pluginName,"Wrong input argument type.");
        return;
    }

    if (p->outputArgCount==1 &&
            p->outputArgTypeAndSize[0]!=sim_lua_arg_float){
        simSetLastError(pluginName,"Wrong output argument type.");
        return;
    }

    // if inputs are correct go ahead
    const int objectHandle = p->inputInt[0];
    const int dataHeader = p->inputInt[1];

    const uint buffSize=simGetObjectCustomDataLength(objectHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);

    if (buffSize<0){
        simSetLastError(pluginName,"The object does not contain any custom data for this plugin.");
        return;
    }

    std::vector<unsigned char> developerCustomData(buffSize);
    simGetObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,(simChar*)developerCustomData.data());

    std::vector<unsigned char> tempData;

    if(CAccess::extractSerializationData(developerCustomData, dataHeader, tempData)){
        p->outputArgCount=1; // 1 return value (function succeeded)
        p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt)); // x return values takes x*2 simInt for the type and size buffer
        p->outputArgTypeAndSize[1]=sim_lua_arg_nil;     // Not used (table size if the return value was a table)

        if (dataHeader == CustomDataHeaders::QUADROTOR_DATA_CTRL_MODE){
            p->outputArgTypeAndSize[0]=sim_lua_arg_int;   // The first return value is a float
            p->outputInt=(simInt*)simCreateBuffer(1*sizeof(simInt)); // 1 float return value
            p->outputInt[0] = (simInt)CAccess::pop_int(tempData);

        } else {
            p->outputArgTypeAndSize[0]=sim_lua_arg_float;   // The first return value is a float
            p->outputFloat=(simFloat*)simCreateBuffer(1*sizeof(simFloat)); // 1 float return value
            p->outputFloat[0] = (simFloat)CAccess::pop_float(tempData);
        }
    } else {
        p->outputArgCount=0; // 1 return value (function succeeded)
        simSetLastError(pluginName,"The data was not found.");
    }
}


simVoid GenericObjectContainer::simExtSetFloatCustomDataFromHeader(SLuaCallBack* p){

    if (p->inputArgCount!=3){
        simSetLastError(pluginName,"This function requires 2 input arguments.");
        return;
    }

    if (p->outputArgCount>1){
        simSetLastError(pluginName,"This function accepts 1 output argument.");
        return;
    }

    if(p->inputArgTypeAndSize[0]!=sim_lua_arg_int || p->inputInt[0]<0 ||
            p->inputArgTypeAndSize[2]!=sim_lua_arg_int ||
            p->inputArgTypeAndSize[4]!=sim_lua_arg_float){
        simSetLastError(pluginName,"Wrong input argument type.");
        return;
    }

    if (p->outputArgCount==1 &&
            p->outputArgTypeAndSize[0]!=sim_lua_arg_float){
        simSetLastError(pluginName,"Wrong output argument type.");
        return;
    }

    // if inputs are correct go ahead
    const int objectHandle = p->inputInt[0];
    const int dataHeader = p->inputInt[1];

    const uint buffSize=simGetObjectCustomDataLength(objectHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);

    if (buffSize<0){
        simSetLastError(pluginName,"Error getting custom data from the object.");
        return;
    }

    std::vector<unsigned char> developerCustomData(buffSize);
    simGetObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,(simChar*)developerCustomData.data());

    // 2. From that retrieved data, extract sub-data with the dataHeader tag, update it, and add it back to the retrieved data:
    std::vector<unsigned char> tempData;
    CAccess::extractSerializationData(developerCustomData, dataHeader, tempData);

    tempData.clear(); // we discard the old value (if present)

    const float dataValue = p->inputFloat[0];
    CAccess::push_float(tempData, dataValue); // we replace it with the new value

    CAccess::insertSerializationData(developerCustomData,dataHeader,tempData);

    // 3. We add/update the scene object with the updated custom data:
    simAddObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,
            (simChar*)developerCustomData.data(),int(developerCustomData.size()));

    ///TODO: add output bool to indicate success
//    std::vector<unsigned char> tempData;
//    if(extractSerializationData(developerCustomData, dataHeader, tempData)){
//
//        p->outputArgCount=1; // 1 return value (function succeeded)
//
//        p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt)); // x return values takes x*2 simInt for the type and size buffer
//
//        p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_float;   // The first return value is an int
//        p->outputArgTypeAndSize[2*0+1]=sim_lua_arg_nil;     // Not used (table size if the return value was a table)
//
//        p->outputFloat=(simFloat*)simCreateBuffer(1*sizeof(simFloat)); // 1 float return value
//        p->outputFloat[0] = (simFloat)CAccess::pop_float(tempData);
//    } else {
//        simSetLastError(pluginName,"The data was not found.");
//    }
}

simVoid GenericObjectContainer::simExtSetFloatArrayCustomDataFromHeader(SLuaCallBack* p){

    if (p->inputArgCount!=3){
        simSetLastError(pluginName,"This function requires 2 input arguments.");
        return;
    }

    if (p->outputArgCount>1){
        simSetLastError(pluginName,"This function accepts 1 output argument.");
        return;
    }

    if(p->inputArgTypeAndSize[0]!=sim_lua_arg_int || p->inputInt[0]<0 ||
            p->inputArgTypeAndSize[2]!=sim_lua_arg_int ||
            p->inputArgTypeAndSize[4]!=(sim_lua_arg_float | sim_lua_arg_table)){
        simSetLastError(pluginName,"Wrong input argument type.");
        return;
    }

    if (p->outputArgCount==1 &&
            p->outputArgTypeAndSize[0]!=sim_lua_arg_float){
        simSetLastError(pluginName,"Wrong output argument type.");
        return;
    }

    // if inputs are correct go ahead
    const int objectHandle = p->inputInt[0];
    const int dataHeader = p->inputInt[1];

    const uint buffSize=simGetObjectCustomDataLength(objectHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);

    if (buffSize<0){
        simSetLastError(pluginName,"Error getting custom data from the object.");
        return;
    }

    std::vector<unsigned char> developerCustomData(buffSize);
    simGetObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,(simChar*)developerCustomData.data());

    // 2. From that retrieved data, extract sub-data with the dataHeader tag, update it, and add it back to the retrieved data:
    std::vector<unsigned char> tempData;
    CAccess::extractSerializationData(developerCustomData, dataHeader, tempData);

    tempData.clear(); // we discard the old value (if present)

//    const float dataValue = p->inputFloat[0];
    std::vector<simFloat> dataValue(p->inputArgTypeAndSize[5]);
    memcpy(dataValue.data(), p->inputFloat, p->inputArgTypeAndSize[5]*sizeof(simFloat));
    CAccess::push_float(tempData, dataValue); // we replace it with the new value

    CAccess::insertSerializationData(developerCustomData,dataHeader,tempData);

    // 3. We add/update the scene object with the updated custom data:
    simAddObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,
            (simChar*)developerCustomData.data(),int(developerCustomData.size()));

    ///TODO: add output bool to indicate success
//    std::vector<unsigned char> tempData;
//    if(extractSerializationData(developerCustomData, dataHeader, tempData)){
//
//        p->outputArgCount=1; // 1 return value (function succeeded)
//
//        p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt)); // x return values takes x*2 simInt for the type and size buffer
//
//        p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_float;   // The first return value is an int
//        p->outputArgTypeAndSize[2*0+1]=sim_lua_arg_nil;     // Not used (table size if the return value was a table)
//
//        p->outputFloat=(simFloat*)simCreateBuffer(1*sizeof(simFloat)); // 1 float return value
//        p->outputFloat[0] = (simFloat)CAccess::pop_float(tempData);
//    } else {
//        simSetLastError(pluginName,"The data was not found.");
//    }
}

simVoid GenericObjectContainer::simExtSetIntCustomDataFromHeader(SLuaCallBack* p){

    if (p->inputArgCount!=3){
        simSetLastError(pluginName,"This function requires 2 input arguments.");
        return;
    }

    if (p->outputArgCount>1){
        simSetLastError(pluginName,"This function accepts 1 output argument.");
        return;
    }

    if(p->inputArgTypeAndSize[0]!=sim_lua_arg_int || p->inputInt[0]<0 ||
            p->inputArgTypeAndSize[2]!=sim_lua_arg_int ||
            p->inputArgTypeAndSize[4]!=sim_lua_arg_int){
        simSetLastError(pluginName,"Wrong input argument type.");
        return;
    }

    if (p->outputArgCount==1 &&
            p->outputArgTypeAndSize[0]!=sim_lua_arg_float){
        simSetLastError(pluginName,"Wrong output argument type.");
        return;
    }

    // if inputs are correct go ahead
    const int objectHandle = p->inputInt[0];
    const int dataHeader = p->inputInt[1];

    const uint buffSize=simGetObjectCustomDataLength(objectHandle, CustomDataHeaders::DEVELOPER_DATA_HEADER);

    if (buffSize<0){
        simSetLastError(pluginName,"Error getting custom data from the object.");
        return;
    }

    std::vector<unsigned char> developerCustomData(buffSize);
    simGetObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,(simChar*)developerCustomData.data());

    // 2. From that retrieved data, extract sub-data with the dataHeader tag, update it, and add it back to the retrieved data:
    std::vector<unsigned char> tempData;
    CAccess::extractSerializationData(developerCustomData, dataHeader, tempData);

    tempData.clear(); // we discard the old value (if present)

    const int dataValue = p->inputInt[2];
    CAccess::push_int(tempData, dataValue); // we replace it with the new value

    CAccess::insertSerializationData(developerCustomData,dataHeader,tempData);

    // 3. We add/update the scene object with the updated custom data:
    simAddObjectCustomData(objectHandle,CustomDataHeaders::DEVELOPER_DATA_HEADER,
            (simChar*)developerCustomData.data(),int(developerCustomData.size()));

    ///TODO: add output bool to indicate success
//    std::vector<unsigned char> tempData;
//    if(extractSerializationData(developerCustomData, dataHeader, tempData)){
//
//        p->outputArgCount=1; // 1 return value (function succeeded)
//
//        p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt)); // x return values takes x*2 simInt for the type and size buffer
//
//        p->outputArgTypeAndSize[2*0+0]=sim_lua_arg_float;   // The first return value is an int
//        p->outputArgTypeAndSize[2*0+1]=sim_lua_arg_nil;     // Not used (table size if the return value was a table)
//
//        p->outputFloat=(simFloat*)simCreateBuffer(1*sizeof(simFloat)); // 1 float return value
//        p->outputFloat[0] = (simFloat)CAccess::pop_float(tempData);
//    } else {
//        simSetLastError(pluginName,"The data was not found.");
//    }
}





