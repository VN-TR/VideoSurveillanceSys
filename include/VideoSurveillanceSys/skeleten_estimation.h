#pragma once

#ifndef __SKELETEN_ESTIMATION_H__
#define __SKELETEN_ESTIMATION_H__


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
#include <stdio.h>
#include <iostream>
#include "Windows.h"
#include <time.h>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <regex>
#include <numeric>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "VideoSurveillanceSys/timer.h"
#include <fstream>

using namespace std;
using namespace cv;



// DECLARATION PART
namespace VisionMonitor
{
	/**
	 * @brief 相机类
	 */
	class SkeletenEstimation
	{
	public:

		SkeletenEstimation();

		~SkeletenEstimation();

		void init();

		void poseEstimatiom(const Mat image);

	private:

		void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

		void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);

		op::Wrapper opWrapper_{ op::ThreadManagerMode::Asynchronous };
		
	};
}





#endif // __SKELETEN_ESTIMATION_H__