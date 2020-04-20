/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
 * @file:  common.cpp
 * @version:  1.0.0
 * @author: ybc
 * @date: 2020.2.19
 * @brief: 公共工具类
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include <chrono>
#include <windows.h>
#include <iostream>

#include "VideoSurveillanceSys/common.h"

// CODE

using namespace std;
double common::deg2rad(float deg)
{
	return deg * 3.14159265358979323846 / 180.0;
}

double common::rad2deg(float rad)
{
	return rad * 180.0 / 3.14159265358979323846;
}



std::string  common::get_time()
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	char tmp[64] = { NULL };
	sprintf_s(tmp, "%4d-%02d-%02d %02d:%02d:%02d.%03d", sys.wYear, sys.wMonth,
		sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
	std::string time_str = tmp;
	return time_str;
}

__int64  common::get_time_stamp()
{
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp;
	tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
	__int64 timestamp = tmp.count();
	return timestamp;
}

std::vector<std::string>  common::split(const std::string& str, const std::string& delim)
{
	std::vector<std::string> res;
	if ("" == str) return res;

	//先将要切割的字符串从string类型转换为char*类型;
	char * strs = new char[str.length() + 1];
	strcpy(strs, str.c_str());

	char * d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());

	char *p = strtok(strs, d);
	while (p) {
		std::string s = p; //分割得到的字符串转换为string类型;
		res.push_back(s); //存入结果数组;
		p = strtok(NULL, d);
	}

	delete[] strs;
	delete[] d;
	delete p;

	return res;
}

