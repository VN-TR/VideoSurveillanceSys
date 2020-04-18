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
#include "VideoSurveillanceSys/monitor.h"
#include "HCNetSDK/HCNetSDK.h"
#include "HCNetSDK/plaympeg4.h"
#include <thread>

// 时间解析宏定义;
#define GET_YEAR(_time_)      (((_time_)>>26) + 2000) 
#define GET_MONTH(_time_)     (((_time_)>>22) & 15)
#define GET_DAY(_time_)       (((_time_)>>17) & 31)
#define GET_HOUR(_time_)      (((_time_)>>12) & 31) 
#define GET_MINUTE(_time_)    (((_time_)>>6)  & 63)
#define GET_SECOND(_time_)    (((_time_)>>0)  & 63)
#define XML_BUF 3*1024*1024
#define IDC_STATIC_PLAY = 1000;


// DECLARATION PART
namespace VisionMonitor
{
	/**
	 * @brief 相机类
	 */
	class Camera
	{
	public:

		Camera();

		~Camera();

		void monitorThread();


		/**
		  * @brief 初始化相机，库位,设置连接时间与重连时间，注册设备，获取设备基本信息，加载训练结果
		  * @return 返回相机初始化结果
		  * @retval true 成功
		  * @retval false 不成功
		  */
		bool initialize(const Params &param);


		void close();
		/**
		* @brief 启动摄像头监测
		* @return 无
		* @retval void
		*/
		std::thread* startMonitor();


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
		  * @brief 获取设备编端口号
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

		/**
		  * @brief 抓拍
		  * @param[in] int ipicType 抓图类型
		  * @param[out] std::string ipicType 图片名称
		  * @return 返回抓拍图片
		  * @retval cv::Mat 抓拍图片
		  */
		cv::Mat grabbingFrame(Params &params, int ipicType, std::string &img_name);

		/**
		  * @brief  获取摄像头当前的图像
		  * @author admin
		  * @date   2018年9月3日
		  * @return cv::Mat  摄像头当前的图像
		  */
		cv::Mat getCameraImage();



		/**
		  * @brief  打印处理过程中间图像
		  * @date   2018年9月20日
		  * @param[in]  int r  图像行
		  * @param[in]  int c  图像列
		  */
		void display(int r, int c, const Params &param);

		/**
		  * @brief  将LOGO插入到图片中
		  * @date   2018年9月20日
		  * @param[in]  int rowStart  图像行
		  * @param[in]  int colStart  图像列
		  */
		void InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);


	private:


		/**
		  * @brief 向设备注册
		  * @return 返回注册结果
		  * @retval ture 成功
		  * @retval false 不成功
		  */
		bool doLogin();

		/**
		  * @brief 获取设备的通道资源
		  * @return 无
		  * @retval void
		  */
		void doGetDeviceResoureCfg();

		/**
		  * @brief 注销登录
		  * @return 无
		  * @retval void
		  */
		void doLoginOut();



	private:

		// 配置参数;
		int							id_;						/*! <编号 */
		int							port_;						/*! <端口号 */
		std::string					ip_;						/*! <IP地址 */
		std::string					user_;					    /*! <用户名 */
		std::string					pwd_;						/*! <密码 */
		

		cv::Mat                     image_;                     /*! <相机图像 */
		cv::Mat                     Title_image;                /*! <标题图片 */
		cv::Mat                     Inform_car_image;           /*! <提示车的图片 */
		cv::Mat                     Inform_human_image;         /*! <提示人的图片 */
		cv::Mat                     Inform_good_image;          /*! <提示货的图片 */
		cv::Mat						shadow_;
		cv::Mat						background_;				/*! <背景图片 */
		cv::Mat						camera_cell_merge_;			/*! <相机背景与库位背景合并图像*/
		cv::Mat						blob_target_;				/*! <带目标的图像 */
		cv::Mat						contours_img_;				/*! <带目标轮廓的图像 */
		cv::Mat						convex_img_;				/*! <目标凸多边形图像 */
		cv::Mat						intrinsic_matrix_;			/*! <摄像头内部参数 */
		cv::Mat						distortion_coeffs_;			/*! <镜头畸变参数 */
		float						background_percentage_;		/*! <背景百分比 */
		float						shadow_percentage_;			/*! <阴影百分比 */
		MonitorState                monitor_action_;            /*! <监测功能 */
		std::vector<Cell>		    vct_cell_;					/*! <库位信息列表 */

		DeeplearningDetector        deeplearning_detector_;     /*! <深度学习 */
		std::vector<Saveditem>      AI_result;                  /*! <深度学习的结果 */
		
		
		// 运行参数;
		bool						isLogin_;					/*! <是否为登录状态 */
		LONG						lPlayHandle_;				/*!< */
		LONG						user_id_;					/*!< */
		LONG						handle_;					/*!< */
		LOCAL_DEVICE_INFO			struDeviceInfo_;			/*!< */
		NET_VCA_FIELDDETECION		struFieldDetection_;		/*!< */
		NET_DVR_CHANNEL_GROUP		struFielDetectionCond_;	    /*!< */
		NET_DVR_SETUPALARM_PARAM	struAlarmParam_;			/*!< */

		Params                      param_;
		bool                        path_loaded_;
		std::vector<std::string>    test_image_path_;
		int                         frame_index_;

		//暂存参数;
		bool                        unsafe_Forklift;            /*! <叉车进入库位 */
		bool                        unsafe_Good;                /*! <货物放歪 */
		bool                        unsafe_Human;               /*! <人进入库位 */



	}; // end class camera

} // end namespace VisionMonitor

#endif	// __CAMERA_H__
