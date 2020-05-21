#pragma once

#ifndef __CAMERA_H__
#define __CAMERA_H__
/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file: camera.hh
* @version: V 1.0.0
* @author: bcyang@Visionnav.com;
* @date: 2020-04-18
* @brief: 相机类头文件;
* @details:
* @verbatim:
*/

// INCLUDE PART
#include <opencv/cv.h>
#include "HCNetSDK/HCNetSDK.h"
#include "HCNetSDK/plaympeg4.h"
#include <thread>
#include <openpose/headers.hpp>
#include "VideoSurveillanceSys/timer.h"
using namespace cv;
typedef HWND(WINAPI *PROCGETCONSOLEWINDOW)();

// DECLARATION PART
namespace VisionMonitor
{

	struct Params
	{
		bool data_collection_stage = false;			/*!< 是否为采集数据阶段 true：图片采集 false：运行阶段 */
		int data_collection_interval = 2000;		/*!< 采集数据间隔 /毫秒 */
		bool image_log_switch = true;		        /*!< 记录图片日志开关 */
		bool display_switch = true;		            /*!< 图像显示输出开关 */
		bool show_FPS = false;						/*!< 打印帧数 */
		bool show_current_time = true;				/*!< 打印当前时间 */
		bool obtain_video = false;				    /*!< 获取视频流信息 */
		int data_from = 1;							/*!< 1：实时 0：静态数据./test_image */
		int	connect_time = 2000;					/*!< 连接时间 */
		int	reconnect_time = 10000;					/*!< 重连时间 */
		int lChannel = 1;							/*!< 预览通道号 */
		int dwStreamType = 0;						/*!< 0 - 主码流，1 - 子码流，2 - 码流3，3 - 码流4，以此类推 */
		int dwLinkMode = 0;							/*!< 0 - TCP 方式，1 - UDP 方式，2 - 多播方式，3 - RTP 方式，4 - RTP / RTSP，5 - RSTP / HTTP */
		int bBlocked = 0;							/*!< 0 - 非阻塞取流，1 - 阻塞取流 */
		int image_input_width = 1920;				/*!< 输入图片宽度 */
		int image_input_height = 1080;				/*!< 输入图片高度 */
		bool image_input_flip = false;				/*!< 输入图片反转 */
		int image_input_flipcode = -1;				/*!< 输入图片反转类型 >0: 沿y-轴翻转, 0: 沿x-轴翻转, <0: x、y轴同时翻转*/
		int image_output_width = 1920;				/*!< 显示图片宽度 */
		int image_output_height = 1080;				/*!< 显示图片高度 */
		int skeleton_desample_rate = 4;				/*!< 骨骼识别降采样率 */
		int object_detect_desample_rate = 2;		/*!< 物体检测降采样率 */
		int obtain_video_FPS = 7;					/*!< 输出视频流FPS */
		int obtain_video_color = 1;					/*!< 输出视频流颜色  0 - 灰度 1 - 彩色 */
		int obtain_video_width = 1920;				/*!< 输出视频流宽度 */
		int obtain_video_height = 1080;				/*!< 输出视频流高度 */
		bool only_show_front = false;				/*!< 只显示前视 */
	};


	/**
	 * @brief 相机类
	 */
	class Camera
	{
	public:

		Camera();

		~Camera();

		/*!
		* @ brief  初始化相机,设置连接时间与重连时间，注册设备，获取设备基本信息
		* @ author ybc
		* @ date   2020年4月20日
		* @ param[in]  const Params & param 输入监控参数
		* @ return     bool  返回相机初始化结果
		* @ note
		*/
		bool initialize(const Params &param);

		/*!
		* @ brief  相机硬件初始化
		* @ author ybc
		* @ date   2020年4月20日
		* @ param[in]  const Params & param 输入监控参数
		* @ return     bool  返回相机初始化结果
		* @ note
		*/
		bool HKinit(const Params &param);


		void HKClean();

		/*!
		* @ brief  抓图
		* @ author ybc
		* @ date   2020年4月29日
		* @ return     std::thread*  
		* @ note
		*/
		std::thread* startGrab();



		/*!
		* @ brief  抓图线程
		* @ author ybc
		* @ date   2020年4月29日
		* @ return     void  
		* @ note
		*/
		void grabThread();


		/*!
		* @ brief  拍一张图片
		* @ author ybc
		* @ date   2020年4月29日
		* @ param[in]  Params & params	参数表
		* @ param[in]  std::string & img_name	图片名
		* @ return     cv::Mat  输出抓到的图片
		* @ note	将流中的图片拍出来
		*/
		cv::Mat grabbingFrame(Params &params, std::string &img_name);

		/**
		* @brief 设置设备编号
		* @param[in] int id 设备编号
		* @return 无
		* @retval void
		*/
		void setID(int id);

		/**
		  * @brief 获取设备编号
		  * @return 返回设备编号
		  * @retval int 设备编号
		  */
		int getID(void);

		/**
		  * @brief 设置设备IP地址
		  * @param[in] std::string ip 设备IP地址
		  * @return 无
		  * @retval void
		  */
		void setIP(std::string ip);

		/**
		  * @brief 获取设备IP地址
		  * @return 返回设备IP地址
		  * @retval std::string 设备IP地址
		  */
		std::string getIP(void);

		/**
		  * @brief 设置设备端口号
		  * @param[in] int port 设备端口号
		  * @return 无
		  * @retval void
		  */
		void setPort(int port);

		/**
		  * @brief 获取设备编号
		  * @return 返回设备端口号
		  * @retval int 设备端口号
		  */
		int getPort(void);

		/**
		  * @brief 设置设备编号
		  * @param[in] std::string user 设备注册用户名
		  * @return 无
		  * @retval void
		  */
		void setUser(std::string user);

		/**
		  * @brief 获取设备注册用户名
		  * @return 返回设备注册用户名
		  * @retval std::string 设备注册用户名
		  */
		std::string getUser(void);

		/**
		  * @brief 设置设备注册密码
		  * @param[in] std::string pwd 设备注册密码
		  * @return 无
		  * @retval void
		  */
		void setPWD(std::string pwd);

		/**
		  * @brief 获取设备注册密码
		  * @return 返回设备注册密码
		  * @retval std::string 设备注册密码
		  */
		std::string getPWD(void);


		/*!
		* @ brief  设置相机在合并图中的位置
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  std::string pwd
		* @ return     void  
		* @ note
		*/
		void setSite(std::string pwd);
	

		/*!
		* @ brief  获取相机在合并图片中的位置
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     std::string  
		* @ note
		*/
		std::string getSite(void);


		/**
		  * @brief  获取摄像头内部参数
		  * @date   2018年9月18日
		  * @return     cv::Mat  内部参数矩阵
		  */
		cv::Mat	getIntrinsicMatrix();


		/**
		  * @brief  设置摄像头内部参数
		  * @date   2018年9月18日
		  * @param[in]  cv::Mat intrinsic_matrix  内部参数矩阵
		  */
		void setIntrinsicMatrix(cv::Mat &intrinsic_matrix);


		/**
		  * @brief  获取镜头畸变矩阵
		  * @author admin
		  * @date   2018年9月18日
		  * @param[out]
		  * @return     cv::Mat  畸变矩阵
		  */
		cv::Mat	getDistortionCoeffs();


		/**
		  * @brief  设置镜头畸变参数矩阵
		  * @author admin
		  * @date   2018年9月18日
		  * @param[in]  cv::Mat & distortion_coeffs  畸变矩阵
		  */
		void setDistortionCoeffs(cv::Mat &distortion_coeffs);


		/*!
		* @ brief  获取上一帧图片
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     cv::Mat  
		* @ note
		*/
		Mat getlastimage();

		bool close_;
		private:
			//相机参数
			int							id_;						/*! <编号 */
			int							port_;						/*! <端口号 */
			std::string					ip_;						/*! <IP地址 */
			std::string					user_;					    /*! <用户名 */
			std::string					pwd_;						/*! <密码 */
			std::string					site_;						/*! <位置 */
			cv::Mat						intrinsic_matrix_;			/*! <摄像头内部参数 */
			cv::Mat						distortion_coeffs_;			/*! <镜头畸变参数 */

			//相机运行
			LONG						lUserID_;					/*! <编号 */
			NET_DVR_DEVICEINFO_V30		struDeviceInfo_;			/*! <设备信息 */
			LONG						lRealPlayHandle_;			/*! <播放句柄 */
			HWND						hWnd_;						/*! <句柄 */

			//配置参数
			Params						param_;						/*! <配置参数表 */
			std::vector<std::string>    test_image_path_;			/*! <离线数据位置 */

			//标志位
			bool                        path_loaded_;
			bool						first_grab_;				/*! <抓第一帧 */	

			//运行参数
			Timer						grab_time_;					/*! <抓图时间 */
			int							frame_index_;				/*! <帧数 */
			cv::Mat						image_;						/*! <处理图像 */

			//线程处理
			std::list<Mat>		        msgRecvQueueGrabeMat_;			/*! <相机捕获的图像队列 */
			std::mutex					grab_image_mutex_;				/*! <输入图片锁 */

	}; // end class camera



} // end namespace VisionMonitor

#endif	// __CAMERA_H__
