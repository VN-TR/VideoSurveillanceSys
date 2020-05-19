#pragma once

#ifndef __OBJECT_DETECTION_H__
#define __OBJECT_DETECTION_H__


/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file: object_detection.hh
* @version: V 1.0.0
* @author: bcyang@Visionnav.com;
* @date: 2020-04-19
* @brief: 物体检测头文件;
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
	//标签结构体;
	struct labelmap
	{
		int id;
		string labelname;
	};

	/**
	 * @brief 物体检测类
	 */
	class ObjectDetection
	{
	public:

		/*!
		* @ brief  物体检测构造函数
		* @ author ybc
		* @ date   2020年4月19日
		* @ return       
		* @ note
		*/
		ObjectDetection();

		/*!
		* @ brief  物体检测析构函数
		* @ author ybc
		* @ date   2020年4月19日
		* @ return       
		* @ note
		*/
		~ObjectDetection();

		/*!
		* @ brief  物体检测初始化
		* @ author ybc
		* @ date   2020年4月19日
		* @ return     void  
		* @ note
		*/
		void Init();

		/*!
		* @ brief  深度学习检测过程
		* @ author ybc
		* @ date   2020年4月19日
		* @ param[in]  cv::Mat & img 输入图片
		* @ return     std::vector<Saveditem>  输出检测结果
		* @ note
		*/
		vector<Saveditem> DL_Detector(const cv::Mat &img_input, const cv::Mat &draw, cv::Mat &img_output);
		
		void ObjectDetection::Release();
	private:

		/*!
		* @ brief  载入标签
		* @ author ybc
		* @ date   2020年4月19日
		* @ return     void  
		* @ note
		*/
		void readLabelMap();
		
		vector<Saveditem>			itemInfomation_;						/*! <深度学习检测结果 */
		vector<labelmap>			label_map_;								/*! <标签 */
		dnn_tensorflow				tfutil_;								/*! <tensorflow封装 */
		std::vector<const char*>	INPUT_NODES_;							/*! <输入节点 */
		std::vector<const char*>	OUTPUT_NODES_;							/*! <输出节点 */
		string						MODEL_NAME_;							/*! <模型地址 */
		string						label_path_;							/*! <标签地址 */
		std::vector<std::int64_t>	input_dims_;							/*! <输入维度 */
		size_t						input_tensor_element_count_;			/*! <输入张量个数 */
		std::vector<std::int64_t>	out_dims_;								/*! <输出维度 */
		TF_Output					input_ops[1];							/*! <输入 */
		TF_Output					output_ops[4];							/*! <输出 */
	};

}

#endif // _OBJECT_DETECTION_H__