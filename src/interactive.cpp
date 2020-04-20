/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
 * @file: interactive.cpp
 * @version:
 * @author: bcyang@Visionnav.com
 * @date:
 * @brief:
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/interactive.h"
#include "VideoSurveillanceSys/monitor.h"

#include <string>
//#include <crtdbg.h>
#include <iostream>

using namespace std;
using namespace VisionMonitor;

// CODE
static Monitor monitor;
//CLog mylog;


//EXTERN_C SURVEILLANCE_API bool __stdcall initMointor()
//{
//	return monitor.initMointor();
//}
//
//EXTERN_C SURVEILLANCE_API bool __stdcall closeMonitor()
//{
//	return monitor.closeMonitor();
//}