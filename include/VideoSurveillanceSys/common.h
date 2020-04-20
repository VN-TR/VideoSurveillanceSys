#ifndef __COMMON_H__
#define __COMMON_H__

/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file:  common.h
* @version: 1.0.0
* @author: ybc
* @date:  2020.4.20
* @brief:  公共工具类
* @details:
* @verbatim:
*/

// INCLUDE PART
#include<iostream>
#include <string.h>
#include <vector>

// DECLARATION PART
/*!
* @brief: 公共工具类
*/
class common
{
public:
	/**
	  * @brief  公共工具类构造函数
	  * @author ybc
	  * @date   2020年2月19日
	  * @param[out] 无
	  * @return  无
	  */
	common() {}

	/**
	  * @brief  公共工具类析造函数
	  * @author ybc
	  * @date   2020年2月19日
	  * @param[out] 无
	  * @return 无
	  */
	~common() {}

	/*!
	* @ brief  角度变弧度
	* @ author ybc
	* @ date   2020年2月19日
	* @ param[in]  float deg   角度值
	* @ return     double      弧度值
	* @ note
	*/
	static double common::deg2rad(float deg);


	/*!
	* @ brief  弧度变角度
	* @ author ybc
	* @ date   2020年2月19日
	* @ param[in]  float rad   弧度值
	* @ return     double      角度值
	* @ note
	*/
	static double common::rad2deg(float rad);



	/**
	  * @brief  获取时间字符串
	  * @author bcyang@VisionNav.com
	  * @date   2020年04月20日
	  * @return     std::string  字符串
	  */
	static std::string get_time();

	/**
	  * @brief  获取时间（毫秒）
	  * @author bcyang@VisionNav.com
	  * @date   2020年04月20日
	  * @return     __int64  时间（毫秒）
	  */
	static __int64 get_time_stamp();

	/**
	  * @brief  按指定字符分割字符串
	  * @author bcyang@VisionNav.com
	  * @date   2020年04月20日
	  * @param[in]  const std::string & str  字符串
	  * @param[in]  const std::string & delim  分割符
	  * @return     std::vector<std::string>  字符串数组
	  */
	static std::vector<std::string> split(const std::string& str, const std::string& delim);



};

#endif	// __COMMON_H__
