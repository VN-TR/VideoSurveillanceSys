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
//* @brief: ����ʶ��ͷ�ļ�;
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
//	 * @brief �����
//	 */
//	class SkeletonEstimation
//	{
//	public:
//
//		
//		/*!
//		* @ brief  ���캯��
//		* @ author ybc
//		* @ date   2020��4��19��
//		* @ return       
//		* @ note
//		*/
//		SkeletonEstimation();
//
//		/*!
//		* @ brief  ��������
//		* @ author ybc
//		* @ date   2020��4��19��
//		* @ return       
//		* @ note
//		*/
//		~SkeletonEstimation();
//
//	//	/*!
//	//	* @ brief  ��ʼ��skeleten����
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void init();
//
//	//	/*!
//	//	* @ brief  ��̬����
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ param[in]  const Mat image  ����һ��ͼƬ
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void poseEstimatiom(const Mat image);
//
//	//	
//	//	/*!
//	//	* @ brief  
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ return     cv::Mat  ��ȡ����ʶ��ͼ��
//	//	* @ note
//	//	*/
//	//	Mat getSkeletonImage();
//
//
//	//	/*!
//	//	* @ brief  
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ return     std::vector<float>  ��ȡ��������
//	//	* @ note
//	//	*/
//	//	vector<float> getSkeletonPoint();
//
//	//	/*!
//	//	* @ brief  
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ return     int  ��ȡʶ���˵ĸ���
//	//	* @ note
//	//	*/
//	//	int getPeopleCount();
//
//	private:
//
//	//	/*!
//	//	* @ brief  openpose�Դ��ĹǼ����ͼ
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ param[in]  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> & datumsPtr ������
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
//
//	//	/*!
//	//	* @ brief  ��ӡ�Ǽ�����
//	//	* @ author ybc
//	//	* @ date   2020��4��19��
//	//	* @ param[in]  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> & datumsPtr ������
//	//	* @ return     void  
//	//	* @ note
//	//	*/
//	//	void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
//		op::Wrapper opWrapper;
//		//op::Wrapper					opWrapper_;							/*! <openpose ��װ */
//		//cv::Mat						skeletonImage_;				        /*! <����ʶ�����ͼ�� */
//		//vector<float>				skeletonPoint_;						/*! <����ʶ��� */
//		//int							peopleCount_;						/*! <ʶ�𵽵����� */
//	};
//}
//
//
//
//
//
//#endif // __SKELETON_ESTIMATION_H__