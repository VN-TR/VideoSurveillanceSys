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
#define SURVEILLANCE_EXPORTS	1				// ����dll�ļ�;
#endif

#ifdef SURVEILLANCE_EXPORTS
#define SURVEILLANCE_API __declspec(dllexport)
#else
#define SURVEILLANCE_API __declspec(dllimport)
#endif


#include <cstdint>


/**
  * @brief  ��������ͷģ�飬��ʼ������ͷ�����ѧϰģ�飬�������ͷ�Ϳ�λ��ʼ����������true�����򷵻�false��
  * @date   2020��4��20��
  */
EXTERN_C SURVEILLANCE_API bool __stdcall initMointor();


/**
  * @brief  �����ʱ�ļ����ͷſռ䣬�رռ����򣬳ɹ�����true, ���򷵻�false
  * @date   2020��4��20��
  * @return �ɹ�����true, ���򷵻�false
  */
EXTERN_C SURVEILLANCE_API bool __stdcall closeMonitor();




#endif	// __INTERACTIVE_H__