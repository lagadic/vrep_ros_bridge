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

#include "vrep_ros_plugin/v_repExtRosBridge.h"
#include "vrep_ros_plugin/access.h"
#include <iostream>
#include "v_repLib.h"

#include <time.h>

#include "vrep_ros_plugin/GenericObjectContainer.h"

#include <ros/ros.h>
#include <boost/bind.hpp>

#ifdef _WIN32
	#include <shlwapi.h> // required for PathRemoveFileSpec function
	#define WIN_AFX_MANAGE_STATE AFX_MANAGE_STATE(AfxGetStaticModuleState())
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#define WIN_AFX_MANAGE_STATE
#endif /* __linux || __APPLE__ */

LIBRARY vrepLib;

static GenericObjectContainer* objectContainer = NULL;

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.
	WIN_AFX_MANAGE_STATE;

	// Dynamically load and bind V-REP functions:
	char curDirAndFile[1024];
#ifdef _WIN32
	GetModuleFileName(NULL,curDirAndFile,1023);
	PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
	if (getcwd(curDirAndFile, sizeof(curDirAndFile))==NULL){
	    // output some error if desired
	}
#endif

	std::string currentDirAndPath(curDirAndFile);
	std::string temp(currentDirAndPath);

#ifdef _WIN32
	temp+="\\v_rep.dll";
#elif defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */

	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load v_rep.dll. Cannot start '" << pluginName << "' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in v_rep.dll. Cannot start '" << pluginName << "' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	// Check the V-REP version:
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<VREP_VERSION_MAJOR*1e4+VREP_VERSION_MINOR*1e2+VREP_VERSION_PATCH)
	{
		std::cout << "Sorry, your V-REP copy is somewhat old, V-REP" <<
		        VREP_VERSION_MAJOR << "." << VREP_VERSION_MINOR << "." << VREP_VERSION_PATCH <<
		        "or higher is required. Cannot start '" << pluginName << "' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}

	//Initialize ROS
	if(!ros::isInitialized()){
	    int argc = 0;
        ros::init(argc,NULL,"vrep");
	}

    if(!ros::master::check())
    {
        std::cout << "ROS master is not running. Cannot start '" << pluginName << "' plugin.\n";
        return (0);
    }


	// Do the normal plugin initialization:
	simLockInterface(1);

    objectContainer = new GenericObjectContainer();

	// Register lua collbacks
	const int inArgsSimExtGetAllCustomData[]={1,sim_lua_arg_int};
	simRegisterCustomLuaFunction("simExtGetAllCustomData",
	        "string customData=simExtGetAllCustomData(number objectHandle)",
	        inArgsSimExtGetAllCustomData, GenericObjectContainer::simExtGetAllCustomData);
    const int inArgsSimExtGetCustomDataFromHeader[]={2,sim_lua_arg_int,sim_lua_arg_int};
    simRegisterCustomLuaFunction("simExtGetCustomDataFromHeader",
            "number data=simExtGetCustomDataFromHeader(number objectHandle, number dataHeader)",
            inArgsSimExtGetCustomDataFromHeader, GenericObjectContainer::simExtGetCustomDataFromHeader);
    const int inArgsSimExtSetFloatCustomDataFromHeader[]={3,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_float};
    simRegisterCustomLuaFunction("simExtSetFloatCustomDataFromHeader",
            "number data=simExtSetFloatCustomDataFromHeader(number objectHandle, number dataHeader, number value)",
            inArgsSimExtSetFloatCustomDataFromHeader, GenericObjectContainer::simExtSetFloatCustomDataFromHeader);
    const int inArgsSimExtSetFloatArrayCustomDataFromHeader[]={3,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_float|sim_lua_arg_table};
    simRegisterCustomLuaFunction("simExtSetFloatArrayCustomDataFromHeader",
            "number data=simExtSetFloatArrayCustomDataFromHeader(number objectHandle, number dataHeader, table value)",
            inArgsSimExtSetFloatArrayCustomDataFromHeader, GenericObjectContainer::simExtSetFloatArrayCustomDataFromHeader);
    const int inArgsSimExtSetIntCustomDataFromHeader[]={3,sim_lua_arg_int,sim_lua_arg_int,sim_lua_arg_int};
    simRegisterCustomLuaFunction("simExtSetIntCustomDataFromHeader",
            "number data=simExtSetIntCustomDataFromHeader(number objectHandle, number dataHeader, number value)",
            inArgsSimExtSetIntCustomDataFromHeader, GenericObjectContainer::simExtSetIntCustomDataFromHeader);

    CustomDataHeaders::registerCustomDataHeaders();




	simLockInterface(0);
	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)

}

VREP_DLLEXPORT void v_repEnd()
{ // This is called just once, at the end of V-REP
	WIN_AFX_MANAGE_STATE;

	delete objectContainer;
	objectContainer=NULL;

	unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
	WIN_AFX_MANAGE_STATE;

	simLockInterface(1);
	static bool refreshDlgFlag=true;

	// This function should not generate any error messages:
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);

	void* retVal=NULL;

	if (message==sim_message_eventcallback_refreshdialogs){
	    // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too
	    refreshDlgFlag=true;
	}


	if (message==sim_message_eventcallback_menuitemselected){
	    // A custom menu bar entry was selected..
	}

	if (message==sim_message_eventcallback_instancepass){
	    // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

		const int flags=auxiliaryData[0];
		const bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
		const bool instanceSwitched=((flags&64)!=0);

		if (instanceSwitched)
		{
			// React to an instance switch here!!
		}

		if (sceneContentChanged)
		{ // we actualize plugin objects for changes in the scene
			objectContainer->actualizeForSceneContent();
			refreshDlgFlag=true;
		}
	}

	if (message==sim_message_eventcallback_moduleopen)
	{ // A script called simOpenModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp(pluginName,(char*)customData)==0) ) // is the command also meant for this plugin?
			objectContainer->startOfSimulation();				// yes!
	}

	if (message==sim_message_eventcallback_modulehandle){
	    // A script called simHandleModule (by default the main script). Is only called during simulation.


		if ( (customData==NULL)||(_stricmp(pluginName,(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			//clock_t init, final;

			//init=clock();
			objectContainer->handleSimulation();
			//final=clock()-init;
			//std::cout << (double)final / ((double)CLOCKS_PER_SEC) << std::endl;
		}// yes!
	}



	if (message==sim_message_eventcallback_moduleclose){
	    // A script called simCloseModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp(pluginName,(char*)customData)==0) ) // is the command also meant for this plugin?
			objectContainer->endOfSimulation();					// yes!
	}

	if (message==sim_message_eventcallback_instanceswitch){
        // Here the user switched the instance. React to this message in a similar way as you would react to a full
        // scene content change. In this plugin example, we react to an instance switch by reacting to the
        // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
        // (see here above)
	}

	if (message==sim_message_eventcallback_broadcast){
	    // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)
	}

	if (message==sim_message_eventcallback_scenesave){
	    // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)
	}

	if (message==sim_message_eventcallback_simulationabouttostart){
	    // Simulation is about to start
	}

	if (message==sim_message_eventcallback_simulationended){
	    // Simulation just ended
	}

	// You can add more messages to handle here

	if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag){
	    // handle refresh of the plugin's dialog:
		refreshDlgFlag=false;
	}

	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	simLockInterface(0);
	return(retVal);
}
