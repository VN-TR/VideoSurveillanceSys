/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
 * @file: camera.cc
 * @version: V 1.0.0
 * @author: xzhang@Visionnav.com;
 * @date: 2018-08-21
 * @brief: ����ຯ��ʵ��;
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/camera.h"
#include "VideoSurveillanceSys/monitor.h"
#include "VideoSurveillanceSys/file_operation.h"
#include <io.h>
#include <assert.h>
#include <windows.h>
#include <opencv2/opencv.hpp>
#include "VideoSurveillanceSys/timer.h"

using namespace std;
// CODE
namespace VisionMonitor
{
	Camera::Camera() {}
	Camera::~Camera() {}
	
	bool Camera::initialize(const Params &param)
	{
		param_ = param;
		object_detection_.Init();
		HKinit(param_);

		return true;
	}

	bool Camera::HKinit(const Params &param)
	{
		NET_DVR_Init();
		//��������ʱ��������ʱ��
		NET_DVR_SetConnectTime(param.connect_time, 1);
		NET_DVR_SetReconnect(param.reconnect_time, true);

		char ip[128], user[128], pwd[128];
		strcpy(ip, ip_.c_str());
		strcpy(user, user_.c_str());
		strcpy(pwd, pwd_.c_str());
		lUserID_ = NET_DVR_Login_V30(ip, port_, user, pwd, &struDeviceInfo_);
		if (lUserID_ < 0)
		{
			printf("Login error, %d\n", NET_DVR_GetLastError());
			NET_DVR_Cleanup();
			return false;
		}
		//����Ԥ�������ûص�������
		hWnd_ = GetConsoleWindow();
		NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
		struPlayInfo.hPlayWnd = hWnd_; //��ҪSDK ����ʱ�����Ϊ��Чֵ����ȡ��������ʱ����Ϊ��
		struPlayInfo.lChannel = 1; //Ԥ��ͨ����
		struPlayInfo.dwStreamType = 0; //0-��������1-��������2-����3��3-����4���Դ�����
		struPlayInfo.dwLinkMode = 0; //0- TCP ��ʽ��1- UDP ��ʽ��2- �ಥ��ʽ��3- RTP ��ʽ��4-RTP/RTSP��5-RSTP/HTTP
		struPlayInfo.bBlocked = 0; //0- ������ȡ����1- ����ȡ��

		lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, NULL, NULL);
		return true;
	}

	void Camera::setID(int id)
	{
		id_ = id;
	}

	int Camera::getID(void)
	{
		return id_;
	}

	void Camera::setIP(std::string ip)
	{
		ip_ = ip;
	}

	std::string Camera::getIP(void)
	{
		return ip_;
	}

	void Camera::setPort(int port)
	{
		port_ = port;
	}

	int Camera::getPort(void)
	{
		return port_;
	}

	void Camera::setUser(std::string user)
	{
		user_ = user;
	}

	std::string Camera::getUser(void)
	{
		return user_;
	}


	void Camera::setPWD(std::string pwd)
	{
		pwd_ = pwd;
	}

	std::string Camera::getPWD(void)
	{
		return pwd_;
	}

	cv::Mat	Camera::getIntrinsicMatrix()
	{
		return intrinsic_matrix_;
	}

	void Camera::setIntrinsicMatrix(cv::Mat &intrinsic_matrix)
	{
		intrinsic_matrix.copyTo(intrinsic_matrix_);
	}

	cv::Mat	Camera::getDistortionCoeffs()
	{
		return distortion_coeffs_;
	}

	void Camera::setDistortionCoeffs(cv::Mat &distortion_coeffs)
	{
		distortion_coeffs.copyTo(distortion_coeffs_);
	}
}