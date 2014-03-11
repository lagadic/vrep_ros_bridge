
#include "vrep_ros_plugin/ConsoleHandler.h"

//#include "porting.h"
//#include "access.h"
#include "v_repLib.h"
#include <iostream>

namespace ConsoleHandler {

	bool ConsoleIsOpen = 0;
	simInt consoleHandle = -1;



   void printInConsole(const std::stringstream  &strStream)
   {
	   // If the console is not open, open it
	   	if (!ConsoleIsOpen){
	   		consoleHandle = simAuxiliaryConsoleOpen("External console",200,4,NULL,NULL,NULL,NULL);
	   		ConsoleIsOpen = 1;
	   	}

	   	// Print the message in the console
	   	std::stringstream  newline;
	   	newline << "-------------------------------------------------------------------" << std::endl;

	   	newline << strStream.str().c_str();

	   	simAuxiliaryConsolePrint(consoleHandle, newline.str().c_str());
	    std::cout << newline.str().c_str();

   }
}
