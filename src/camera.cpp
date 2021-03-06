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
	Camera::Camera() :frame_index_(0), path_loaded_(0),first_grab_(false),close_(false){}
	Camera::~Camera() {}
	
	


	bool Camera::initialize(const Params &param)
	{
		param_ = param;

		if (param_.data_from == OffLine)
		{
			std::string test_video_dir = "./test_image/camera" + std::to_string(getID()) + "/1.avi";
			cap_ =  VideoCapture(test_video_dir);
		}
		cout << "start monitor init" << endl;

		//如果使用实时数据时,初始化相机硬件
		if (param_.data_from == OnLine)
		{
			cout << "camera sdk init start" << endl;
			HKinit(param_);
			cout << "camera sdk init end" << endl;
		}
		//如果是数据采集阶段，并且采集的是视频的话，则创建视频文件
		if (param_.data_collection_stage && param_.data_collection_mp4)
		{
			char VideoName[256] = { 0 };
			int64_t time = common::getSysTimeMicros();
			sprintf_s(VideoName, ".\\image_log\\camera%d\\%I64d.avi", id_, time);
			writer.open(VideoName, VideoWriter::fourcc('M', 'J', 'P', 'G'), 20,
				Size(1920, 1080), 1);
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

	cv::Mat Camera::grab_image_from_avi()
	{
		cv::Mat frame_f;
		cap_.read(frame_f);
		return frame_f;
	}


	void Camera::grabThread()
	{

		while (!close_)
		{
			grab_time_.tic();
			std::string pic_name;
			Mat grabimg = grabbingFrame(param_, pic_name);
			if (grabimg.data != NULL)
			{	
				image_ = grabimg;
				{
					std::lock_guard<std::mutex> locker_image_(grab_image_mutex_);
					msgRecvQueueGrabeMat_.push_back(grabimg);
					if (msgRecvQueueGrabeMat_.size() > 2)
					{
						msgRecvQueueGrabeMat_.pop_front();
					}
				}
				cout << "camera " << getID() << " 抓图时间: " << grab_time_.toc() << "ms" << endl;
				if (first_grab_ == false)
					waitKey(5000);
				first_grab_ = true;
			}
			//如果是离线跑视频（屏蔽）
			if (param_.data_from == 0)
			{
				/*Sleep(400);*/
			}
			//如果在线
			else
			{
				if (param_.data_collection_stage)
					Sleep(10);
				else
					Sleep(50);
			}
		}

		if (param_.data_collection_stage && param_.data_collection_mp4)
		{
			writer.release();
		}
	}



	cv::Mat Camera::grabbingFrame(Params &params,  std::string &img_name)
	{
		char PicName[256] = { 0 };
		char sJpegPicBuffer[1024] = { 0 };
		cv::Mat img, org;
		frame_index_++;

		//如果是离线且第一个则读取全部
		//if (0== param_.data_from && !path_loaded_)
		//{
		//	path_loaded_ = true;
		//	FileOperation               file_opt;
		//	std::string test_image_dir = "./test_image/camera" + std::to_string(getID());
		//	cout << test_image_dir << endl;
		//	file_opt.getFileNameList(test_image_dir, ".jpg", test_image_path_);
		//}
		

		cv::Mat org_image;
		
		//如果是离线,离线不负责采集程序，这里主要是离线图片读取（暂时弃用）
		if (0== param_.data_from)
		{
				//if (frame_index_ >= test_image_path_.size()) return img;
				//std::string  img_path = "./test_image/camera" + std::to_string(getID()) + "/" + to_string(frame_index_) + ".jpg";
				//org_image = cv::imread(img_path);
				//img = org_image;
		}
		//在线
		else
		{
			int64_t time = common::getSysTimeMicros();
			LONG iCurChan = lRealPlayHandle_;
			sprintf_s(PicName, ".\\image_log\\camera%d\\%I64d_ch%02ld.jpg", id_, time, iCurChan);

			BYTE *pBuffer1 = new BYTE[1920 * 1080 * 1.5];
			DWORD dwBufSize1 = 1920 * 1080*1.5;
			DWORD dwBufSize2;
			
			//在线视频流抓图
			if (PlayM4_GetJPEG(lRealPlayHandle_, pBuffer1, dwBufSize1, &dwBufSize2))
			{
				//解码
				cv::Mat rawData(1, dwBufSize2, CV_8UC1, (void*)pBuffer1);
				cv::Mat decodedImage = cv::imdecode(rawData, 1);

				//如果不为空，则处理后输出
				if (decodedImage.data != NULL)
				{
					img = decodedImage;	
					//如果需要反转
					if (param_.image_input_flip)
					{
						flip(img,img, param_.image_input_flipcode);
					}
					//如果需要调整大小
					if (param_.image_input_height != 1080)
					{
						resize(img, img, Size(param_.image_input_width, param_.image_input_height));
					}

					//数据采集
					//如果是数据采集阶段、且采集类型为视频
					if (param_.data_collection_stage && param_.data_collection_mp4)
					{
						string cameraid = to_string(id_);
						imshow(cameraid, img);
						waitKey(1);
						writer.write(img);
					}
					//如果不是视频
					else if (params.data_collection_stage && param_.data_collection_mp4 == false)
					{
						string cameraid = to_string(id_);
						imshow(cameraid, img);
						imwrite(PicName, img);						
						Sleep(params.data_collection_interval);
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


	void Camera::HKClean()
	{
		NET_DVR_Logout(lUserID_);
		NET_DVR_Cleanup();
	}
}