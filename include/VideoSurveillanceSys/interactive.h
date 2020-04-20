#ifndef __INTERACTIVE_H__
#define __INTERACTIVE_H__
/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file: interactive.h
* @version:
* @author: bcyang@Visionnav.com
* @date:
* @brief:
* @details:
* @verbatim:
*/

// INCLUDE PART
#include <windows.h>

// DECLARATION PART
#ifndef SURVEILLANCE_EXPORTS
#define SURVEILLANCE_EXPORTS	1				// 配置dll文件;
#endif

#ifdef SURVEILLANCE_EXPORTS
#define SURVEILLANCE_API __declspec(dllexport)
#else
#define SURVEILLANCE_API __declspec(dllimport)
#endif


#include <cstdint>


/**
  * @brief  启动摄像头模块，初始化摄像头和深度学习模块，如果摄像头和库位初始化正常返回true，否则返回false。
  * @date   2020年4月20日
  */
EXTERN_C SURVEILLANCE_API bool __stdcall initMointor();


/**
  * @brief  清除临时文件，释放空间，关闭监测程序，成功返回true, 否则返回false
  * @date   2020年4月20日
  * @return 成功返回true, 否则返回false
  */
EXTERN_C SURVEILLANCE_API bool __stdcall closeMonitor();




#endif	// __INTERACTIVE_H__