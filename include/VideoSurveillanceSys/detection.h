//#pragma once
//
//#ifndef __DETECTION_H__
//#define __DETECTION_H__
///**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
//* @file: detection.h
//* @version: V 1.0.0
//* @author: bcyang@Visionnav.com;
//* @date: 2020-05-06
//* @brief: �����ͷ�ļ�;
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
//* @ brief  ����߳�
//* @ author ybc
//* @ date   2020��5��6��
//* @ return     void
//* @ note	���������⼰�������
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
//		* @ brief  �����������������
//		* @ author ybc
//		* @ date   2020��5��6��
//		* @ param[in]  const Mat input_image
//		* @ return     std::vector<float>  
//		* @ note	����caffe����GPU�����㡣��������openpose���ԣ�ʹ�ü���ʱ����ͣ�����̡߳�
//		*/
//		vector<float> skeleton_estimation(const Mat input_image);
//
//
//		/*!
//		* @ brief  ���ƹ���ʶ��ͼ
//		* @ author ybc
//		* @ date   2020��4��24��
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
//		Timer						detect_time_;				/*! <������ʱ�� */
//		Timer						skeleton_time_;				/*! <�������ʱ�� */
//		Timer						display_time_;				/*! <ͼƬ��ʾʱ�� */
//
//		//������
//		ObjectDetection				object_detection_;			/*! <��������� */
//		std::vector<Saveditem>      AI_result_;					/*! <�������� */
//		std::vector<float>			skeleton_point_;			/*! <��������� */
//		int							skeleton_people_count_;		/*! <����������� */
//
//
//		//���ò���
//		Params						param_;						/*! <���ò����� */
//		std::vector<std::string>    test_image_path_;			/*! <��������λ�� */
//
//		//��־λ
//		bool                        path_loaded_;
//
//
//		//���в���
//		int							frame_index_;				/*! <֡�� */
//		cv::Mat						image_;						/*! <����ͼ�� */
//		cv::Mat						display_image_;				/*! <��ʾͼ�� */
//		cv::Mat						skeleton_image_;			/*! <����ͼ�� */
//		cv::Mat						Title_image_;				/*! <��ǩͼ�� */
//		cv::Mat						Inform_car_image_;			/*! <��ǩͼ�� */
//		cv::Mat						Inform_human_image_;		/*! <��ǩͼ�� */
//		cv::Mat						Inform_good_image_;			/*! <��ǩͼ�� */
//		cv::Mat						map_image_;					/*! <��ͼ */
//
//		//�̴߳���
//		std::list<Mat>		        msgRecvQueueMat_;			/*! <��������ͼ����� */
//		std::mutex					image_mutex_;				/*! <����ͼƬ�� */
//		std::list<Mat>		        msgQueue_AI_Mat_;			/*! <AI���ͼ����� */
//		std::mutex					AI_image_mutex_;			/*! <AI���ͼƬ�� */
//		std::mutex					AI_res_mutex_;				/*! <AI����� */
//		std::list<vector<Saveditem>>msgQueue_Ai_Result_;		/*! <AI������� */
//		std::mutex					Skeleton_res_mutex_;		/*! <��������� */
//		std::list<vector<float>>	msgQueue_skeleton_Result_;	/*! <AI������� */
//
//		std::mutex					have_people_mutex_;			/*! <�Ƿ������� */
//		std::list<bool>				msgQueue_have_people_;		/*! <�Ƿ����˽������ */
//
//	};
//
//
//
//}
//
//#endif	// __CAMERA_H__