//#pragma once
//
//#ifndef __SKELETON_ESTIMATION_H__
//#define __SKELETON_ESTIMATION_H__
//
//
///**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
//* @file: skeleton_estimation.hh
//* @version: V 1.0.0
//* @author: bcyang@Visionnav.com;
//* @date: 2020-04-18
//* @brief: 骨骼识别头文件;
//* @details:
//* @verbatim:
//*/
//
//// INCLUDE PART
//#include <stdio.h>
//#include <iostream>
//#include <time.h>
//#include <openpose/headers.hpp>
//#include <regex>
//#include <numeric>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include "VideoSurveillanceSys/timer.h"
//#include <fstream>
//
//using namespace std;
//using namespace cv;
//
//
//
//// DECLARATION PART
//namespace VisionMonitor
//{
//	/**
//	 * @brief 相机类
//	 */
//	class SkeletonEstimation
//	{
//	public:
//
//		
//		/*!
//		* @ brief  构造函数
//		* @ author ybc
//		* @ date   2020年4月19日
//		* @ return       
//		* @ note
//		*/
//		SkeletonEstimation();
//
//		/*!
//		* @ brief  析构函数
//		* @ author ybc
//		* @ date   2020年4月19日
//		* @ return       
//		* @ note
//		*/
//		~SkeletonEstimation();
//
//	//	/*!
//	//	* @ brief  初始化skeleten计算
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void init();
//
//	//	/*!
//	//	* @ brief  姿态计算
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ param[in]  const Mat image  输入一张图片
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void poseEstimatiom(const Mat image);
//
//	//	
//	//	/*!
//	//	* @ brief  
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ return     cv::Mat  获取骨骼识别图像
//	//	* @ note
//	//	*/
//	//	Mat getSkeletonImage();
//
//
//	//	/*!
//	//	* @ brief  
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ return     std::vector<float>  获取骨骼坐标
//	//	* @ note
//	//	*/
//	//	vector<float> getSkeletonPoint();
//
//	//	/*!
//	//	* @ brief  
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ return     int  获取识别到人的个数
//	//	* @ note
//	//	*/
//	//	int getPeopleCount();
//
//	private:
//
//	//	/*!
//	//	* @ brief  openpose自带的骨架输出图
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ param[in]  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> & datumsPtr 计算结果
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
//
//	//	/*!
//	//	* @ brief  打印骨架坐标
//	//	* @ author ybc
//	//	* @ date   2020年4月19日
//	//	* @ param[in]  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> & datumsPtr 计算结果
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
//		op::Wrapper opWrapper;
//		//op::Wrapper					opWrapper_;							/*! <openpose 封装 */
//		//cv::Mat						skeletonImage_;				        /*! <骨骼识别输出图像 */
//		//vector<float>				skeletonPoint_;						/*! <骨骼识别点 */
//		//int							peopleCount_;						/*! <识别到的人数 */
//	};
//}
//
//
//
//
//
//#endif // __SKELETON_ESTIMATION_H__