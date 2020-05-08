/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
 * @file: camera.cc
 * @version: V 1.0.0
 * @author: bcyang@Visionnav.com;
 * @date: 2020-05-06
 * @brief: 检测类函数实现;
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/camera.h"
#include "VideoSurveillanceSys/monitor.h"
#include "VideoSurveillanceSys/file_operation.h"
#include "VideoSurveillanceSys/common.h"
#include <chrono>

#include <io.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include "VideoSurveillanceSys/timer.h"
using namespace std;
using namespace std::chrono;
// CODE
namespace VisionMonitor
{
	Camera::Camera() :frame_index_(0), path_loaded_(0),first_grab_(false){}
	Camera::~Camera() {}
	
	


	bool Camera::initialize(const Params &param)
	{
		param_ = param;

		//如果是不是图片采集阶段（运行阶段）则初始化物体检测和骨骼检测
		if (param_.data_collection_stage == false)
		{

		}
		//如果是图片采集阶段或运行阶段使用实时数据时,初始化相机硬件
		if (param_.data_collection_stage == true || param_.data_from)
		{
			HKinit(param_);
		}

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
		struPlayInfo.lChannel = param.lChannel; //预览通道号
		struPlayInfo.dwStreamType = param.dwStreamType; //0-主码流，1-子码流，2-码流3，3-码流4，以此类推
		struPlayInfo.dwLinkMode = param.dwLinkMode; //0- TCP 方式，1- UDP 方式，2- 多播方式，3- RTP 方式，4-RTP/RTSP，5-RSTP/HTTP
		struPlayInfo.bBlocked = param.bBlocked; //0- 非阻塞取流，1- 阻塞取流

		lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, NULL, NULL);
		return true;
	}

	std::thread* Camera::startGrab()
	{
		return new std::thread(&Camera::grabThread, this);
	}

	void Camera::grabThread()
	{
		while (true)
		{
			grab_time_.tic();
			std::string pic_name;
			Mat grabimg = grabbingFrame(param_, pic_name);
			if (grabimg.data != NULL)
			{
				Mat distortimg;
				cv::undistort(grabimg, distortimg, getIntrinsicMatrix(), getDistortionCoeffs());
				grabimg = distortimg;
				image_ = distortimg;
				/*image_ = grabimg;*/
				{
					std::lock_guard<std::mutex> locker_image(image_mutex_);
					msgRecvQueueMat_.push_back(grabimg);
					if (msgRecvQueueMat_.size() > 2)
					{
						msgRecvQueueMat_.pop_front();
					}
				}
				cout << "camera " << getID() << " 抓图时间: " << grab_time_.toc() << "ms" << endl;
				if (first_grab_ == false)
					waitKey(4000);
				first_grab_ = true;

			}
			if (param_.data_from == 0)
			{
				Sleep(200);
			}
			else
			{
				Sleep(10);
			}
	
		}
	}



	cv::Mat Camera::grabbingFrame(Params &params,  std::string &img_name)
	{
		char PicName[256] = { 0 };
		char sJpegPicBuffer[1024] = { 0 };
		cv::Mat img, org;
		frame_index_++;
		if (0== param_.data_from && !path_loaded_)
		{
			path_loaded_ = true;
			FileOperation               file_opt;
			std::string test_image_dir = "./test_image/camera" + std::to_string(getID());
			cout << test_image_dir << endl;
			file_opt.getFileNameList(test_image_dir, ".jpg", test_image_path_);
		}
		

		cv::Mat org_image;
		// read image
		if (0== param_.data_from)
		{
			if (frame_index_ >= test_image_path_.size()) return img;
			std::string  img_path = "./test_image/camera" + std::to_string(getID()) + "/" + test_image_path_[frame_index_];
			org_image = cv::imread(img_path);
			img = org_image;
		}
		else
		{
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
					if (param_.image_input_flip)
					{
						flip(img,img, param_.image_input_flipcode);
					}
					resize(img, img, Size(param_.image_input_width, param_.image_input_height));
					if (params.image_log_switch && !params.data_collection_stage)
					{
						imwrite(PicName, img);
					}
					else if (params.data_collection_stage)
					{
						string cameraid = to_string(id_);
						imshow(cameraid, img);
						imwrite(PicName, img);
						
						waitKey(params.data_collection_interval);
					}
				}
			};
			delete[] pBuffer1;
		}

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

	void Camera::setSite(std::string site)
	{
		site_ = site;
	}

	std::string Camera::getSite(void)
	{
		return site_;
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