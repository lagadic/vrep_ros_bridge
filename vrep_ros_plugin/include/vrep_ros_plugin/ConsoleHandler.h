#ifndef __CONSOLE_HANDLER_H__
#define __CONSOLE_HANDLER_H__


#include <sstream>

/**
 * This namespace contains the function to create and print an external console in V-rep
 */
namespace ConsoleHandler
{
/**
 * This function print a messagge in the external console. If it is the first message, the console is created.
 */
   void printInConsole(const std::stringstream  &strStream);

}

#endif //ndef __CONSOLE_HANDLER_H__
