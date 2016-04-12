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

#include "vrep_ros_plugin/porting.h"

#ifdef _WIN32
DWORD getTimeInMs(void)
{
	return(timeGetTime()&0x03ffffff);
}
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
DWORD getTimeInMs(void)
{
	struct timeval tv;
	DWORD result=0;
	if (gettimeofday(&tv,NULL)==0)
		result=(tv.tv_sec*1000+tv.tv_usec/1000)&0x03ffffff;
	return(result);
}
#endif /* __linux || __APPLE__ */

DWORD getTimeDiffInMs(DWORD lastTime)
{
	DWORD currentTime=getTimeInMs();
	if (currentTime<lastTime)
		return(currentTime+0x03ffffff-lastTime);
	return(currentTime-lastTime);
}
