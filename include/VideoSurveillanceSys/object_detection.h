#pragma once

#ifndef __OBJECT_DETECTION_H__
#define __OBJECT_DETECTION_H__


/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file: object_detection.hh
* @version: V 1.0.0
* @author: bcyang@Visionnav.com;
* @date: 2020-04-19
* @brief: ������ͷ�ļ�;
* @details:
* @verbatim:
*/

// INCLUDE PART
#include <stdio.h>
#include <iostream>
#include "Windows.h"
#include <regex>
#include <numeric>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "VideoSurveillanceSys/dnn_tensorflow.hpp"
#include "VideoSurveillanceSys/timer.h"
#include "VideoSurveillanceSys/saveditem.h"
#include <fstream>

using namespace std;
using namespace cv;



// DECLARATION PART
namespace VisionMonitor
{
	//��ǩ�ṹ��;
	struct labelmap
	{
		int id;
		string labelname;
	};

	/**
	 * @brief ��������
	 */
	class ObjectDetection
	{
	public:

		/*!
		* @ brief  �����⹹�캯��
		* @ author ybc
		* @ date   2020��4��19��
		* @ return       
		* @ note
		*/
		ObjectDetection();

		/*!
		* @ brief  ��������������
		* @ author ybc
		* @ date   2020��4��19��
		* @ return       
		* @ note
		*/
		~ObjectDetection();

		/*!
		* @ brief  �������ʼ��
		* @ author ybc
		* @ date   2020��4��19��
		* @ return     void  
		* @ note
		*/
		void Init();

		/*!
		* @ brief  ���ѧϰ������
		* @ author ybc
		* @ date   2020��4��19��
		* @ param[in]  cv::Mat & img ����ͼƬ
		* @ return     std::vector<Saveditem>  ��������
		* @ note
		*/
		vector<Saveditem> DL_Detector(const cv::Mat &img_input, const cv::Mat &draw, cv::Mat &img_output);
		
		void ObjectDetection::Release();
	private:

		/*!
		* @ brief  �����ǩ
		* @ author ybc
		* @ date   2020��4��19��
		* @ return     void  
		* @ note
		*/
		void readLabelMap();
		
		vector<Saveditem>			itemInfomation_;						/*! <���ѧϰ����� */
		vector<labelmap>			label_map_;								/*! <��ǩ */
		dnn_tensorflow				tfutil_;								/*! <tensorflow��װ */
		std::vector<const char*>	INPUT_NODES_;							/*! <����ڵ� */
		std::vector<const char*>	OUTPUT_NODES_;							/*! <����ڵ� */
		string						MODEL_NAME_;							/*! <ģ�͵�ַ */
		string						label_path_;							/*! <��ǩ��ַ */
		std::vector<std::int64_t>	input_dims_;							/*! <����ά�� */
		size_t						input_tensor_element_count_;			/*! <������������ */
		std::vector<std::int64_t>	out_dims_;								/*! <���ά�� */
		TF_Output					input_ops[1];							/*! <���� */
		TF_Output					output_ops[4];							/*! <��� */
	};

}

#endif // _OBJECT_DETECTION_H__