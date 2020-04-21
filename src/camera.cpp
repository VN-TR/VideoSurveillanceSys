/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
 * @file: camera.cc
 * @version: V 1.0.0
 * @author: xzhang@Visionnav.com;
 * @date: 2018-08-21
 * @brief: 相机类函数实现;
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/camera.h"
#include "VideoSurveillanceSys/monitor.h"
#include "VideoSurveillanceSys/file_operation.h"
#include "VideoSurveillanceSys/common.h"

#include <io.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include "VideoSurveillanceSys/timer.h"
#include "VideoSurveillanceSys/skeleton_estimation.h"

using namespace std;
// CODE
namespace VisionMonitor
{
	Camera::Camera():frame_index_(0) {}
	Camera::~Camera() {}
	
	bool Camera::initialize(const Params &param)
	{
		param_ = param;
		object_detection_.Init();

		Title_image_ = cv::imread("./inform_image/1.png", CV_LOAD_IMAGE_UNCHANGED);
		Inform_car_image_ = cv::imread("./inform_image/2.png", CV_LOAD_IMAGE_UNCHANGED);
		Inform_human_image_ = cv::imread("./inform_image/3.png", CV_LOAD_IMAGE_UNCHANGED);
		Inform_good_image_ = cv::imread("./inform_image/4.png", CV_LOAD_IMAGE_UNCHANGED);

		HKinit(param_);

		return true;
	}

	bool Camera::HKinit(const Params &param)
	{
		NET_DVR_Init();
		//设置连接时间与重连时间
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
		//启动预览并设置回调数据流
		hWnd_ = GetConsoleWindow();
		NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
		struPlayInfo.hPlayWnd = hWnd_; //需要SDK 解码时句柄设为有效值，仅取流不解码时可设为空
		struPlayInfo.lChannel = 1; //预览通道号
		struPlayInfo.dwStreamType = 0; //0-主码流，1-子码流，2-码流3，3-码流4，以此类推
		struPlayInfo.dwLinkMode = 0; //0- TCP 方式，1- UDP 方式，2- 多播方式，3- RTP 方式，4-RTP/RTSP，5-RSTP/HTTP
		struPlayInfo.bBlocked = 0; //0- 非阻塞取流，1- 阻塞取流

		lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, NULL, NULL);
		return true;
	}

	std::thread* Camera::startMonitor()
	{
		//if (path_loaded_ && frame_index_ >= test_image_path_.size())
		//{
		//	return nullptr;
		//}
		return new std::thread(&Camera::monitorThread, this);
	}

	void Camera::monitorThread()
	{
		std::string pic_name;
		image_ = grabbingFrame(param_, pic_name);
		if (image_.data != NULL)
		{
			AI_result_ = object_detection_.DL_Detector(image_);
			if (AI_result_.size() != 0)
			{
				image_ = AI_result_.back().itemImage;
				AI_result_.clear();
			}
		}
	}

	cv::Mat Camera::grabbingFrame(Params &params,  std::string &img_name)
	{
		char PicName[256] = { 0 };
		char sJpegPicBuffer[1024] = { 0 };
		cv::Mat img, org;

		int64_t time = common::get_time_stamp();
		LONG iCurChan = lRealPlayHandle_;
		sprintf_s(PicName, ".\\image_log\\camera%d\\%I64d_ch%02ld.jpg", id_, time, iCurChan);

		BYTE *pBuffer1 = new BYTE[3000 * 2000];
		DWORD dwBufSize1 = 1920 * 1080 * 1.5;
		DWORD dwBufSize2;

		if (PlayM4_GetJPEG(lRealPlayHandle_, pBuffer1, dwBufSize1, &dwBufSize2))
		{
			cv::Mat rawData(1, dwBufSize2, CV_8UC1, (void*)pBuffer1);
			cv::Mat decodedImage = cv::imdecode(rawData, 1);
			if (decodedImage.data != NULL)
			{
				img = decodedImage;
				if (params.image_log_switch)
				{
					if (frame_index_ % 10)
					{
						//imwrite(PicName, img);
					}
					
					frame_index_++;
				}
			}

		};
		delete[] pBuffer1;

		return img;
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

	Mat Camera::getlastimage()
	{
		return image_;
	}

}