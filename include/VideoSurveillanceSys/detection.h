//#pragma once
//
//#ifndef __DETECTION_H__
//#define __DETECTION_H__
///**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
//* @file: detection.h
//* @version: V 1.0.0
//* @author: bcyang@Visionnav.com;
//* @date: 2020-05-06
//* @brief: 检测类头文件;
//* @details:
//* @verbatim:
//*/
//
//// INCLUDE PART
//#include <opencv/cv.h>
//#include "HCNetSDK/HCNetSDK.h"
//#include "HCNetSDK/plaympeg4.h"
//#include "VideoSurveillanceSys/camera.h"
//#include "VideoSurveillanceSys/object_detection.h"
//#include <openpose/headers.hpp>
//#include <thread>
//
//
//// DECLARATION PART
//namespace VisionMonitor
//{
//	class Detection
//	{
//	public:
//		Detection();
//		~Detection();
//
//		void init();
//
//		std::thread* Detection::startDetect();
//		/*!
//* @ brief  检测线程
//* @ author ybc
//* @ date   2020年5月6日
//* @ return     void
//* @ note	包含物体检测及骨骼检测
//*/
//		void detectThread(Mat &input);
//		void detect();
//
//	private:
//
//
//		void display(Mat &object_detect_outimg, vector<float> &skeleton_res, vector<Saveditem> &AI_result);
//
//		void filter(vector<float> &skeleton_res, vector<Saveditem> &AI_result);
//
//		void InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);
//
//
//
//		
//		/*!
//		* @ brief  对人体骨骼进行评估
//		* @ author ybc
//		* @ date   2020年5月6日
//		* @ param[in]  const Mat input_image
//		* @ return     std::vector<float>  
//		* @ note	基于caffe，在GPU下运算。但是由于openpose特性，使得计算时，暂停其他线程。
//		*/
//		vector<float> skeleton_estimation(const Mat input_image);
//
//
//		/*!
//		* @ brief  绘制骨骼识别图
//		* @ author ybc
//		* @ date   2020年4月24日
//		* @ param[in]  const Mat input_image
//		* @ param[out] Mat & output_image
//		* @ return     cv::Mat
//		* @ note
//		*/
//		Mat draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint);
//
//
//
//
//		Timer						detect_time_;				/*! <物体检测时间 */
//		Timer						skeleton_time_;				/*! <骨骼检测时间 */
//		Timer						display_time_;				/*! <图片显示时间 */
//
//		//检测相关
//		ObjectDetection				object_detection_;			/*! <物体检测对象 */
//		std::vector<Saveditem>      AI_result_;					/*! <物体检测结果 */
//		std::vector<float>			skeleton_point_;			/*! <骨骼检测结果 */
//		int							skeleton_people_count_;		/*! <骨骼检测人数 */
//
//
//		//配置参数
//		Params						param_;						/*! <配置参数表 */
//		std::vector<std::string>    test_image_path_;			/*! <离线数据位置 */
//
//		//标志位
//		bool                        path_loaded_;
//
//
//		//运行参数
//		int							frame_index_;				/*! <帧数 */
//		cv::Mat						image_;						/*! <处理图像 */
//		cv::Mat						display_image_;				/*! <显示图像 */
//		cv::Mat						skeleton_image_;			/*! <骨骼图像 */
//		cv::Mat						Title_image_;				/*! <标签图像 */
//		cv::Mat						Inform_car_image_;			/*! <标签图像 */
//		cv::Mat						Inform_human_image_;		/*! <标签图像 */
//		cv::Mat						Inform_good_image_;			/*! <标签图像 */
//		cv::Mat						map_image_;					/*! <地图 */
//
//		//线程处理
//		std::list<Mat>		        msgRecvQueueMat_;			/*! <相机捕获的图像队列 */
//		std::mutex					image_mutex_;				/*! <输入图片锁 */
//		std::list<Mat>		        msgQueue_AI_Mat_;			/*! <AI结果图像队列 */
//		std::mutex					AI_image_mutex_;			/*! <AI结果图片锁 */
//		std::mutex					AI_res_mutex_;				/*! <AI结果锁 */
//		std::list<vector<Saveditem>>msgQueue_Ai_Result_;		/*! <AI结果队列 */
//		std::mutex					Skeleton_res_mutex_;		/*! <骨骼结果锁 */
//		std::list<vector<float>>	msgQueue_skeleton_Result_;	/*! <AI结果队列 */
//
//		std::mutex					have_people_mutex_;			/*! <是否有人锁 */
//		std::list<bool>				msgQueue_have_people_;		/*! <是否有人结果队列 */
//
//	};
//
//
//
//}
//
//#endif	// __CAMERA_H__