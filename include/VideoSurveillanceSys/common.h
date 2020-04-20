#ifndef __COMMON_H__
#define __COMMON_H__

/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file:  common.h
* @version: 1.0.0
* @author: ybc
* @date:  2020.4.20
* @brief:  ����������
* @details:
* @verbatim:
*/

// INCLUDE PART
#include<iostream>
#include <string.h>
#include <vector>

// DECLARATION PART
/*!
* @brief: ����������
*/
class common
{
public:
	/**
	  * @brief  ���������๹�캯��
	  * @author ybc
	  * @date   2020��2��19��
	  * @param[out] ��
	  * @return  ��
	  */
	common() {}

	/**
	  * @brief  �������������캯��
	  * @author ybc
	  * @date   2020��2��19��
	  * @param[out] ��
	  * @return ��
	  */
	~common() {}

	/*!
	* @ brief  �Ƕȱ仡��
	* @ author ybc
	* @ date   2020��2��19��
	* @ param[in]  float deg   �Ƕ�ֵ
	* @ return     double      ����ֵ
	* @ note
	*/
	static double common::deg2rad(float deg);


	/*!
	* @ brief  ���ȱ�Ƕ�
	* @ author ybc
	* @ date   2020��2��19��
	* @ param[in]  float rad   ����ֵ
	* @ return     double      �Ƕ�ֵ
	* @ note
	*/
	static double common::rad2deg(float rad);



	/**
	  * @brief  ��ȡʱ���ַ���
	  * @author bcyang@VisionNav.com
	  * @date   2020��04��20��
	  * @return     std::string  �ַ���
	  */
	static std::string get_time();

	/**
	  * @brief  ��ȡʱ�䣨���룩
	  * @author bcyang@VisionNav.com
	  * @date   2020��04��20��
	  * @return     __int64  ʱ�䣨���룩
	  */
	static __int64 get_time_stamp();

	/**
	  * @brief  ��ָ���ַ��ָ��ַ���
	  * @author bcyang@VisionNav.com
	  * @date   2020��04��20��
	  * @param[in]  const std::string & str  �ַ���
	  * @param[in]  const std::string & delim  �ָ��
	  * @return     std::vector<std::string>  �ַ�������
	  */
	static std::vector<std::string> split(const std::string& str, const std::string& delim);



};

#endif	// __COMMON_H__
