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
#include "VideoSurveillanceSys/skeleton_estimation.h"

// CODE
namespace VisionMonitor
{
	SkeletonEstimation::SkeletonEstimation():opWrapper_(op::ThreadManagerMode::Asynchronous)
	{
		
	}

	SkeletonEstimation::~SkeletonEstimation()
	{

	}

	void SkeletonEstimation::init()
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

	void SkeletonEstimation::display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
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

	void SkeletonEstimation::printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
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

	void SkeletonEstimation::poseEstimatiom(const Mat image)
	{
		auto datumProcessed = opWrapper_.emplaceAndPop(image);
		if (datumProcessed != nullptr)
		{
			auto s = datumProcessed->at(0)->poseScores.toString();
			s.erase(s.find_last_not_of(" "));

			peopleCount_ = datumProcessed->at(0)->poseKeypoints.getSize(0);
			skeletonPoint_.clear();
			skeletonPoint_.resize(peopleCount_ * 3);
			for (int i = 0; i < peopleCount_*75 ; i++)
			{
				skeletonPoint_[i] = datumProcessed->at(0)->poseKeypoints[i];
			}
			Mat skeletonImage_ = datumProcessed->at(0)->cvOutputData;
			display(datumProcessed);
		}
		else
		{
			op::log("Image could not be processed.", op::Priority::High);
		}
	}


	Mat SkeletonEstimation::getSkeletonImage()
	{
		return skeletonImage_;
	}

	vector<float> SkeletonEstimation::getSkeletonPoint()
	{
		return skeletonPoint_;
	}

	int SkeletonEstimation::getPeopleCount()
	{
		return peopleCount_;
	}

}


