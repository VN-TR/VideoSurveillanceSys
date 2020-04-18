/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
 * @file: steketen_estimation.cpp
 * @version: V 1.0.0
 * @author: xzhang@Visionnav.com;
 * @date: 2020-04-18
 * @brief: 相机类函数实现;
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/skeleten_estimation.h"

// CODE
namespace VisionMonitor
{
	SkeletenEstimation::SkeletenEstimation():opWrapper_(op::ThreadManagerMode::Asynchronous)
	{
		
	}

	SkeletenEstimation::~SkeletenEstimation()
	{

	}

	void SkeletenEstimation::init()
	{
		op::log("Starting OpenPose demo...", op::Priority::High);

		// Configuring OpenPose
		op::log("Configuring OpenPose...", op::Priority::High);

		// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
		if (FLAGS_disable_multi_thread)
			opWrapper_.disableMultiThreading();

		// Starting OpenPose
		op::log("Starting thread(s)...", op::Priority::High);
		opWrapper_.start();
	}

	void SkeletenEstimation::display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
	{
		try
		{
			if (datumsPtr != nullptr && !datumsPtr->empty())
			{
				Mat skeleten = datumsPtr->at(0)->cvOutputData;
				resize(skeleten, skeleten, Size(skeleten.cols, skeleten.rows));
				// Display image
				cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", skeleten);
				cv::waitKey(1);
			}
			else
				op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
		}
		catch (const std::exception& e)
		{
			op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
		}
	}

	void SkeletenEstimation::printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
	{
		try
		{
			// Example: How to use the pose keypoints
			if (datumsPtr != nullptr && !datumsPtr->empty())
			{
				// Alternative 1
				op::log("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString());
				cout << datumsPtr->at(0)->poseKeypoints.getSize(0) << endl;
			}
			else
				op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
		}
		catch (const std::exception& e)
		{
			op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
		}
	}

	void SkeletenEstimation::poseEstimatiom(const Mat image)
	{
		auto datumProcessed = opWrapper_.emplaceAndPop(image);
		if (datumProcessed != nullptr)
		{
			auto s = datumProcessed->at(0)->poseScores.toString();
			s.erase(s.find_last_not_of(" "));

			//printKeypoints(datumProcessed);
			//std::cout << "左脚:" << endl;
			//std::cout << datumProcessed->at(0)->poseKeypoints[57] << " , "
			//	<< datumProcessed->at(0)->poseKeypoints[58] << " , "
			//	<< datumProcessed->at(0)->poseKeypoints[59] << std::endl;
			//std::cout << "右脚:" << endl;
			//std::cout << datumProcessed->at(0)->poseKeypoints[69] << " , "
			//	<< datumProcessed->at(0)->poseKeypoints[70] << " , "
			//	<< datumProcessed->at(0)->poseKeypoints[71] << std::endl;

			display(datumProcessed);
		}
		else
		{
			op::log("Image could not be processed.", op::Priority::High);
		}
	}

}


