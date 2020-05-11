#include "VideoSurveillanceSys/monitor.h"
#include "VideoSurveillanceSys/file_operation.h"
#include "VideoSurveillanceSys/common.h"
#include "VideoSurveillanceSys/detection.h"
#include "tinyxml2/tinyxml2.h"
#include <assert.h>
#include <corecrt_io.h>



using namespace std;

namespace VisionMonitor
{

	Monitor::Monitor()
	{

	}
	Monitor::~Monitor()
	{

	}

	op::Wrapper opWrapper{ op::ThreadManagerMode::Asynchronous };

	bool Monitor::initiate()
	{
		frame_count_ = 0;
		object_detection_.Init();
		opWrapper.start();
		opWrapper.disableMultiThreading();

		Title_image_ = cv::imread("./inform_image/title.png", CV_LOAD_IMAGE_UNCHANGED);
		map_image_ = cv::imread("./inform_image/videomap.png", CV_LOAD_IMAGE_UNCHANGED);

		FileOperation fileopt;

		if (!loadParames())
		{
			return false;
		}

		fileopt.checkAndCreateDir(".\\image_log");

		for (size_t i = 0; i < cameras_.size(); i++)
		{
			
			fileopt.checkAndCreateDir(".\\image_log\\camera" + std::to_string(cameras_[i]->getID()));

			std::cout << "camera[" << cameras_[i]->getID() << "] initialize..." << std::endl;
			if (cameras_[i]->initialize(param_))
			{
				std::cout << "OK" << std::endl;
			}
			else
			{
				std::cout << "Fail" << std::endl;
				return false;
			}

		}


		return true;
	}

	void Monitor::start()
	{
		if (monitorThread_.get_id() == std::thread::id())
		{
			is_start = true;
			monitorThread_ = std::thread(&Monitor::monitorThread, this);
		}
	}
	
	void Monitor::construct_input_img(Mat &input_img)
	{
		
		for (auto camera : cameras_)
		{
			string site = camera->getSite();
			Mat camera_img = camera->getlastimage();
			if (site == "TL" && camera_img.data != NULL)
			{
				cv::Mat imageROI;
				imageROI = input_img(cv::Rect(0, 0, 1920, 1080));
				camera_img.copyTo(imageROI);
			}
			if (site == "BL" && camera_img.data != NULL)
			{
				cv::Mat imageROI;
				imageROI = input_img(cv::Rect(0, 1080, 1920, 1080));
				camera_img.copyTo(imageROI);
			}
			if (site == "BR" && camera_img.data != NULL)
			{
				cv::Mat imageROI;
				imageROI = input_img(cv::Rect(1920, 1080, 1920, 1080));
				camera_img.copyTo(imageROI);
			}
		}
	}
	std::thread* Monitor::startFusion()
	{
		return new std::thread(&Monitor::fusionThread, this);
	}

	void Monitor::fusionThread()
	{
		while (true)
		{
			Mat image = Mat(2160, 3840, CV_8UC3, cvScalar(255, 255, 255));
			construct_input_img(image);
			Mat cal_AI_image = image.clone();
			resize(cal_AI_image, cal_AI_image, Size(cal_AI_image.cols / param_.object_detect_desample_rate,
				cal_AI_image.rows / param_.object_detect_desample_rate));
			Mat cal_Ske_image = image.clone();
			resize(cal_Ske_image, cal_Ske_image, Size(cal_Ske_image.cols / param_.skeleton_desample_rate,
				cal_Ske_image.rows / param_.skeleton_desample_rate));

			{
				std::lock_guard<std::mutex> locker_image(image_mutex_);
				msgRecvQueueMat_.push_back(image);
				if (msgRecvQueueMat_.size() > 2)
				{
					msgRecvQueueMat_.pop_front();
				}
			}
			{
				std::lock_guard<std::mutex> locker_Cal_AI_image(Cal_AI_image_mutex_);
				msgRecvQueue_Cal_AI_Mat_.push_back(cal_AI_image);
				if (msgRecvQueue_Cal_AI_Mat_.size() > 2)
				{
					msgRecvQueue_Cal_AI_Mat_.pop_front();
				}
			}
			{
				std::lock_guard<std::mutex> locker_cal_Ske_image(Cal_Ske_image_mutex_);
				msgRecvQueue_Cal_Ske_Mat_.push_back(cal_Ske_image);
				if (msgRecvQueue_Cal_Ske_Mat_.size() > 2)
				{
					msgRecvQueue_Cal_Ske_Mat_.pop_front();
				}
			}

			if (image.data != NULL)
			{
				resize(image, image, Size(960, 540));
				imshow("1", image);
				waitKey(1);
			}
			Sleep(10);
		}
	}

	std::thread* Monitor::startDisplay()
	{
		return new std::thread(&Monitor::displayThread, this);
	}

	void Monitor::displayThread()
	{	
		namedWindow("VideoSurveillanceSys", CV_WINDOW_NORMAL);
		cvResizeWindow("VideoSurveillanceSys", 1280, 760);
		while (true)
		{
			Mat display_image;
			vector<Saveditem> AI_result;
			vector<float> skeleton_res;
			{
				std::lock_guard<std::mutex> locker_AI_image(AI_image_mutex_);
				if (!msgRecvQueue_AI_Mat_.empty())
				{
					display_image = msgRecvQueue_AI_Mat_.back();
				}
			}
			{
				std::lock_guard<std::mutex> locker_AI_res(AI_Res_mutex_);
				if (!msgRecvQueue_AI_Res_.empty())
				{
					AI_result = msgRecvQueue_AI_Res_.back();
				}
			}
			{
				std::lock_guard<std::mutex> locker_Skele_Res(Skele_Res_mutex_);
				if (!msgRecvQueue_Skele_Res_.empty())
				{
					skeleton_res = msgRecvQueue_Skele_Res_.back();
				}
			}

			if (AI_result.size()>0 || frame_count_ >2)
			{		
				Timer displaytime;
				displaytime.tic();
				display(display_image, skeleton_res, AI_result);
				cout << "显示:" << displaytime.toc() << endl;
				Sleep(5);
			}
			Sleep(5);
		}
	}



	std::thread* Monitor::startDetect()
	{
		return new std::thread(&Monitor::detectThread, this);
	}
	void Monitor::detectThread()
	{
		while (true)
		{
			Mat img;
			{
				std::lock_guard<std::mutex> locker_image(image_mutex_);
				if (!msgRecvQueueMat_.empty())
				{
					img = msgRecvQueueMat_.back();
				}
			}
			Mat cal_AI_image;
			{
				std::lock_guard<std::mutex> locker_Cal_AI_image(Cal_AI_image_mutex_);
				if (!msgRecvQueue_Cal_AI_Mat_.empty())
				{
					cal_AI_image = msgRecvQueue_Cal_AI_Mat_.back();
				}
			}
			Mat cal_Ske_image;
			{
				std::lock_guard<std::mutex> locker_cal_Ske_image(Cal_Ske_image_mutex_);
				if (!msgRecvQueue_Cal_Ske_Mat_.empty())
				{
					cal_Ske_image = msgRecvQueue_Cal_Ske_Mat_.back();
				}
			}
			image_ = img;
			if (image_.data != NULL)
			{
				total_detect_time_.tic();
				detect(image_,cal_AI_image,cal_Ske_image);
				cout << "检测总耗时" << total_detect_time_.toc() << endl;
			}
		}
	}

	void Monitor::detect(Mat &input, Mat &AI_input, Mat &Ske_input)
	{
		if (input.data != NULL)
		{
			detect_time_.tic();
			vector<Saveditem> AI_result;
			Mat display_image;
			AI_result = object_detection_.DL_Detector(AI_input, input, display_image);
			AI_result_ = AI_result;
			Mat object_out_img = display_image;
			cout << "检测计算:" << detect_time_.toc() << endl;
			bool havepeople = false;
			for (auto res : AI_result)
			{
				if (res.itemClass == "Human")
				{
					havepeople = true;
					break;
				}
			}
			vector<float> skeleton_res;
			
			if (havepeople)
			{
				if (input.data != NULL)
				{
					skeleton_res = skeleton_estimation(Ske_input);
				}
			}
			vector<float> ske_out = skeleton_res;
			float cal_time = detect_time_.toc();
			cout << "骨骼计算:" <<  cal_time << endl;
			{
				std::lock_guard<std::mutex> locker_time(time_mutex_);
				msgRecvQueue_time_.push_back(cal_time);
				if (msgRecvQueue_time_.size() > 2)
				{
					msgRecvQueue_time_.pop_front();
				}
			}
			{
				std::lock_guard<std::mutex> locker_AI_image(AI_image_mutex_);
				msgRecvQueue_AI_Mat_.push_back(object_out_img);
				if (msgRecvQueue_AI_Mat_.size() > 2)
				{
					msgRecvQueue_AI_Mat_.pop_front();
				}
			}
			{
				std::lock_guard<std::mutex> locker_AI_res(AI_Res_mutex_);
				msgRecvQueue_AI_Res_.push_back(AI_result_);
				if (msgRecvQueue_AI_Res_.size() > 2)
				{
					msgRecvQueue_AI_Res_.pop_front();
				}
			}
			{
				std::lock_guard<std::mutex> locker_Skele_Res(Skele_Res_mutex_);
				msgRecvQueue_Skele_Res_.push_back(ske_out);
				if (msgRecvQueue_Skele_Res_.size() > 2)
				{
					msgRecvQueue_Skele_Res_.pop_front();
				}
			}
		}
	}

	void Monitor::display(const Mat &object_detect_outimg, 
		const vector<float> &skeleton_res, const vector<Saveditem> &AI_result)
	{
		if (frame_count_ > 2) 
		{
			Mat outimage = object_detect_outimg;
			vector<float> skeleton_filter_res;
			if (!skeleton_res.empty())
			{
				skeleton_filter_res = filter(skeleton_res,AI_result);
				Mat skeleton_img = draw_skeleton_image(outimage, skeleton_filter_res);
				outimage = skeleton_img;
			}
			outimage = drawmap(outimage, skeleton_filter_res, AI_result);

			Mat out_img = Mat(2300, 3840, CV_8UC3, cvScalar(255, 255, 255));
			Mat imageROI = out_img(cv::Rect(0, 140, 3840, 2160));
			outimage.copyTo(imageROI);
			cv::cvtColor(out_img, out_img, cv::COLOR_BGR2BGRA);
			InsertLogo(out_img, map_image_, 140, 1920);
			InsertLogo(out_img, Title_image_, 0, 0);
			float cal_time;
			{
				std::lock_guard<std::mutex> locker_time(time_mutex_);
				if (!msgRecvQueue_time_.empty())
				{
					cal_time = msgRecvQueue_time_.back();
				}
			}
			float fps_f = 1000 / cal_time;
			string fps_s = "Fps" + to_string(fps_f);
			fps_s.erase(8);
			Point txt_pt(3300, 100);
			putText(out_img, fps_s, txt_pt, FONT_HERSHEY_COMPLEX, 3, Scalar(0, 0, 255), 6, 8, 0);
			resize(out_img, out_img, Size(param_.image_output_width, param_.image_output_height));
			namedWindow("VideoSurveillanceSys", CV_WINDOW_NORMAL);
			imshow("VideoSurveillanceSys", out_img);
			waitKey(1);
			Sleep(10);
		}
		cout << "frame_count_" << frame_count_ << endl;
		frame_count_++;

	}

	Mat Monitor::drawmap(const Mat &displayimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result)
	{
		Mat outimage = displayimg;
		if (!AI_result.empty())
		{
			for (auto res : AI_result)
			{
				if (res.itemClass == "Forklift")
				{
					//后视
					if (res.itemSite_X2 < 1925 && res.itemSite_Y2 < 1085)
					{

						float va = (res.itemSite_X1 + res.itemSite_X2) / 2;
						float vb = res.itemSite_Y2;
						float A = -0.036;
						float B = 9.9;
						float C = 2.45;
						float D = -14.972;
						float kx, ky;
						kx = (va - 935.5) / 1164;
						ky = (vb - 517.8) / 1164;
						float z = -(D) / (A*kx + B * ky + C);
						float x = kx * z;

						Point camera_pt(2760, 550);
						Point pts;
						if (abs(x) < 10 && abs(z) < 20)
						{
							pts.x = 2760 - z * 40;
							pts.y = 550 - x * 40;
							circle(outimage, pts, 28, Scalar(0, 0, 0), -1);
						}
					}
					//前左
					if (res.itemSite_X2 < 1925 && res.itemSite_Y1 > 1075)
					{	
						float va = (res.itemSite_X1 + res.itemSite_X2) / 2;
						float vb = res.itemSite_Y2;
						float A = 0.0741;
						float B = 8.5722;
						float C = 3.35173;
						float D = -15.8004;
						float kx, ky;
						kx = (va - 961.5) / 1113;
						ky = (vb - 524.8-1080) / 1114;
						float z = -(D) / (A*kx + B * ky + C);
						float x = kx * z;
						float theta1 = acos(x / sqrt(x*x + z * z));
						float theta2 = theta1 + 0.2526;
						x = cos(theta2) * sqrt(x*x + z * z);
						z = sin(theta2) * sqrt(x*x + z * z);
						Point camera_pt(3000, 550);
						Point pts;
						if (abs(x) < 10 && abs(z) < 20)
						{
							pts.x = 3200 + z*40;
							pts.y = 550 + x*40;
							circle(outimage, pts, 28, Scalar(0, 0, 0), -1);
						}
					}
					//前右
					if (res.itemSite_X1 > 1915 && res.itemSite_Y1 > 1075)
					{
						float va = (res.itemSite_X1 + res.itemSite_X2) / 2;
						float vb = res.itemSite_Y2;
						float A = 0.0741;
						float B = 8.5722;
						float C = 3.35173;
						float D = -15.8004;
						float kx, ky;
						kx = (va - 961.5 -1920) / 1113;
						ky = (vb - 524.8 - 1080) / 1114;
						float z = -(D) / (A*kx + B * ky + C);
						float x = kx * z;
						float theta1 = acos(x / sqrt(x*x + z * z));
						float theta2 = theta1 - 0.3867;
						x = cos(theta2) * sqrt(x*x + z * z);
						z = sin(theta2) * sqrt(x*x + z * z);
						Point camera_pt(3000, 550);
						Point pts;
						if (abs(x) < 10 && abs(z) < 20)
						{
							pts.x = 3200 + z * 40;
							pts.y = 550 + x * 40;
							circle(outimage, pts, 28, Scalar(0, 0, 0), -1);
						}
					}
				}
			}
		}

		if (!skeleton_res.empty())
		{
			int humanCount = skeleton_res.size() / 75;

			for (auto i = 0; i < humanCount; i++)
			{
				bool filter_complete = false;
				Point pt_right(0, 0);
				if (skeleton_res[i * 75 + 14 * 3] != 0)
				{
					pt_right = Point((int)skeleton_res[i * 75 + 14 * 3] * param_.skeleton_desample_rate,
						(int)skeleton_res[i * 75 + 14 * 3 + 1] * param_.skeleton_desample_rate);
				}
				Point pt_left(0, 0);
				if (skeleton_res[i * 75 + 11 * 3] != 0)
				{
					pt_left = Point((int)skeleton_res[i * 75 + 11 * 3] * param_.skeleton_desample_rate,
						(int)skeleton_res[i * 75 + 11 * 3 + 1] * param_.skeleton_desample_rate);
				}
				Point pt_human(0, 0);
				if (pt_right.x != 0 && pt_left.x != 0)
				{
					pt_human.x = (pt_left.x + pt_right.x) / 2;
					pt_human.y = (pt_left.y + pt_right.y) / 2;
				}
				else if (pt_left.x != 0)
				{
					pt_human.x = (pt_left.x) / 2;
					pt_human.y = (pt_left.y) / 2;
				}
				else if (pt_right.x != 0)
				{
					pt_human.x = (pt_right.x) / 2;
					pt_human.y = (pt_right.y) / 2;
				}
				//后视
				if (pt_human.x < 1920 && pt_human.y < 1080)
				{
					float va = pt_human.x;
					float vb = pt_human.y;
					for (auto res : AI_result)
					{
						if (res.itemClass == "Forklift")
						{
							if (res.itemSite_X2 < 1925 && res.itemSite_Y2 < 1085)
							{
								float width = res.itemSite_X2 - res.itemSite_X1;
								float hight = res.itemSite_Y2 - res.itemSite_Y1;
								if (va < (res.itemSite_X2 + width/5) && va >(res.itemSite_X1 -width/5)
									&& vb < (res.itemSite_Y2 + hight/5) && vb >(res.itemSite_Y1-hight/5))
								{
									va = (res.itemSite_X1 + res.itemSite_X2) / 2;
									vb = res.itemSite_Y2;
								}
							}
						}
					}

					float A = -0.036;
					float B = 9.9;
					float C = 2.45;
					float D = -14.972;

					float kx, ky;
					kx = (va - 935.5) / 1164;
					ky = (vb - 517.8) / 1164;
					float z = -(D) / (A*kx + B * ky + C);
					float x = kx * z;

					Point camera_pt(2760, 550);
					Point pts;
					if (abs(x) < 10 && abs(z) < 20)
					{
						pts.x = 2760 - z * 40;
						pts.y = 550 - x * 40;
						circle(outimage, pts, 12, Scalar(0, 0, 255), -1);
					}
				}

				//前左
				if (pt_human.x < 1920 && pt_human.y > 1080)
				{
					float va = pt_human.x;
					float vb = pt_human.y;
					for (auto res : AI_result)
					{
						if (res.itemClass == "Forklift")
						{
							if (res.itemSite_X2 < 1925 && res.itemSite_Y1 > 1075)
							{						
								float width = res.itemSite_X2 - res.itemSite_X1;
								float hight = res.itemSite_Y2 - res.itemSite_Y1;
								if (va < (res.itemSite_X2 + width / 5) && va >(res.itemSite_X1 - width / 5)
									&& vb < (res.itemSite_Y2 + hight / 5) && vb >(res.itemSite_Y1 - hight / 5))
								{

									va = (res.itemSite_X1 + res.itemSite_X2) / 2;
									vb = res.itemSite_Y2;
								}
							}
						}
					}

					float A = 0.0741;
					float B = 8.5722;
					float C = 3.35173;
					float D = -15.8004;
					float kx, ky;
					kx = (va - 961.5) / 1113;
					ky = (vb - 524.8 - 1080) / 1114;
					float z = -(D) / (A*kx + B * ky + C);
					float x = kx * z;
					float theta1 = acos(x / sqrt(x*x + z * z));
					float theta2 = theta1 + 0.2526;
					x = cos(theta2) * sqrt(x*x + z * z);
					z = sin(theta2) * sqrt(x*x + z * z);
					Point camera_pt(3000, 550);
					Point pts;
					if (abs(x) < 10 && abs(z) < 20)
					{
						pts.x = 3200 + z * 40;
						pts.y = 550 + x * 40;
						circle(outimage, pts, 12, Scalar(0, 0, 255), -1);
					}
				}

				//前右
				if (pt_human.x > 1920 && pt_human.y > 1080)
				{
					float va = pt_human.x;
					float vb = pt_human.y;
					for (auto res : AI_result)
					{
						if (res.itemClass == "Forklift")
						{
							if (res.itemSite_X1 > 1915 && res.itemSite_Y1 > 1075)
							{
								float width = res.itemSite_X2 - res.itemSite_X1;
								float hight = res.itemSite_Y2 - res.itemSite_Y1;
								if (va < (res.itemSite_X2 + width / 5) && va >(res.itemSite_X1 - width / 5)
									&& vb < (res.itemSite_Y2 + hight / 5) && vb >(res.itemSite_Y1 - hight / 5))
								{
									va = (res.itemSite_X1 + res.itemSite_X2) / 2;
									vb = res.itemSite_Y2;
								}
							}
						}
					}

					float A = 0.0741;
					float B = 8.5722;
					float C = 3.35173;
					float D = -15.8004;
					float kx, ky;
					kx = (va - 961.5 - 1920) / 1113;
					ky = (vb - 524.8 - 1080) / 1114;
					float z = -(D) / (A*kx + B * ky + C);
					float x = kx * z;
					float theta1 = acos(x / sqrt(x*x + z * z));
					float theta2 = theta1 - 0.3867;
					x = cos(theta2) * sqrt(x*x + z * z);
					z = sin(theta2) * sqrt(x*x + z * z);
					Point camera_pt(3000, 550);
					Point pts;
					if (abs(x) < 10 && abs(z) < 20)
					{
						pts.x = 3200 + z * 40;
						pts.y = 550 + x * 40;
						circle(outimage, pts, 12, Scalar(0, 0, 255), -1);
					}
				}
			}
		}
		return outimage;
	}

	vector<float> Monitor::filter(const vector<float> &skeleton_res,const vector<Saveditem> &AI_result)
	{
		vector<float> skeleton_filter_res;
		int humanCount = skeleton_res.size() / 75;

		for (auto i = 0; i < humanCount; i++)
		{
			vector<float> human_point;
			float skeleton_x = 0;
			float skeleton_y = 0;
			int skeleton_count = 0;
			for (auto j = 0; j < 25; j++)
			{
				human_point.push_back(skeleton_res[i * 75 + j * 3]);
				human_point.push_back(skeleton_res[i * 75 + j * 3 + 1]);
				human_point.push_back(skeleton_res[i * 75 + j * 3 + 2]);
				if (skeleton_res[i * 75 + j * 3] != 0)
				{
					skeleton_x += skeleton_res[i * 75 + j * 3] * param_.skeleton_desample_rate;
					skeleton_y += skeleton_res[i * 75 + j * 3 + 1 ] * param_.skeleton_desample_rate;
					skeleton_count++;
				}	
			}
			skeleton_x /= skeleton_count;
			skeleton_y /= skeleton_count;
			for (auto res : AI_result)
			{
				if (res.itemClass == "Human")
				{
					if (skeleton_x<res.itemSite_X2 && skeleton_x > res.itemSite_X1
						&& skeleton_y<res.itemSite_Y2 && skeleton_y > res.itemSite_Y1)
					{
						for (int a =0 ;a < 75;a++)
						{
							skeleton_filter_res.push_back(human_point[a]);
						}
						human_point.clear();
						break;
					}
				}
			}
		}
		return skeleton_filter_res;
	}


	vector<float> Monitor::skeleton_estimation(const Mat input_image)
	{
	
		vector<float> skeleton_point;
		auto datumProcessed = opWrapper.emplaceAndPop(input_image);
		if (datumProcessed != nullptr)
		{
			auto s = datumProcessed->at(0)->poseScores.toString();
			s.erase(s.find_last_not_of(" "));

			int peopleCount = datumProcessed->at(0)->poseKeypoints.getSize(0);
			skeleton_people_count_ = peopleCount;
			skeleton_point.clear();
			skeleton_point.resize(peopleCount * 75);
			for (int i = 0; i < peopleCount * 75; i++)
			{
				skeleton_point[i] = datumProcessed->at(0)->poseKeypoints[i];
			}

		}
		return skeleton_point;
	}


	Mat Monitor::draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint)
	{
		Mat outputimage = input_image;

		int humanCount = skeletonPoint.size() / 75;
		int lineWidth = 6;
		int pointWidth = 8;
		Timer mytime;
		mytime.tic();
		for (auto i = 0; i < humanCount; i++)
		{
			//画节点；
			//头部
			//0-鼻子
			Point pt0(0, 0);
			if (skeletonPoint[i * 75] != 0)
			{
				pt0 = Point((int)skeletonPoint[i * 75] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt0, pointWidth, Scalar(74, 38, 252), -1);
			}
			//1-Neck
			Point pt1(0, 0);
			if (skeletonPoint[i * 75 + 1 * 3] != 0)
			{
				pt1 = Point((int)skeletonPoint[i * 75 + 1 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 1 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt1, pointWidth, Scalar(31, 42, 244), -1);
			}
			//15-REye
			Point pt15(0, 0);
			if (skeletonPoint[i * 75 + 15 * 3] != 0)
			{
				pt15 = Point((int)skeletonPoint[i * 75 + 15 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 15 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt15, pointWidth, Scalar(171, 26, 246), -1);
			}
			//16-LEye
			Point pt16(0, 0);
			if (skeletonPoint[i * 75 + 16 * 3] != 0)
			{
				pt16 = Point((int)skeletonPoint[i * 75 + 16 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 16 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt16, pointWidth, Scalar(170, 16, 74), -1);
			}
			//17-REar
			Point pt17(0, 0);
			if (skeletonPoint[i * 75 + 17 * 3] != 0)
			{
				pt17 = Point((int)skeletonPoint[i * 75 + 17 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 17 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt17, pointWidth, Scalar(171, 26, 246), -1);
			}
			//18-LEar
			Point pt18(0, 0);
			if (skeletonPoint[i * 75 + 18 * 3] != 0)
			{
				pt18 = Point((int)skeletonPoint[i * 75 + 18 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 18 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt18, pointWidth, Scalar(170, 16, 74), -1);
			}
			if (pt0.x != 0 && pt1.x != 0)
			{
				line(outputimage, pt0, pt1, Scalar(74, 38, 252), lineWidth);
			}
			if (pt0.x != 0 && pt15.x != 0)
			{
				line(outputimage, pt0, pt15, Scalar(171, 26, 246), lineWidth);
			}
			if (pt0.x != 0 && pt16.x != 0)
			{
				line(outputimage, pt0, pt16, Scalar(170, 16, 74), lineWidth);
			}
			if (pt17.x != 0 && pt15.x != 0)
			{
				line(outputimage, pt17, pt15, Scalar(171, 26, 246), lineWidth);
			}
			if (pt18.x != 0 && pt16.x != 0)
			{
				line(outputimage, pt18, pt16, Scalar(170, 16, 74), lineWidth);
			}

			//右臂
			//2-RShoulder
			Point pt2(0, 0);
			if (skeletonPoint[i * 75 + 2 * 3] != 0)
			{
				pt2 = Point((int)skeletonPoint[i * 75 + 2 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 2 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt2, pointWidth, Scalar(1, 126, 242), -1);
			}
			//3-RElbow
			Point pt3(0, 0);
			if (skeletonPoint[i * 75 + 3 * 3] != 0)
			{
				pt3 = Point((int)skeletonPoint[i * 75 + 3 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 3 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt3, pointWidth, Scalar(0, 203, 253), -1);
			}
			//4-RWrist
			Point pt4(0, 0);
			if (skeletonPoint[i * 75 + 4 * 3] != 0)
			{
				pt4 = Point((int)skeletonPoint[i * 75 + 4 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 4 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt4, pointWidth, Scalar(67, 238, 245), -1);
			}
			if (pt1.x != 0 && pt2.x != 0)
			{
				line(outputimage, pt1, pt2, Scalar(1, 126, 242), lineWidth);
			}
			if (pt2.x != 0 && pt3.x != 0)
			{
				line(outputimage, pt2, pt3, Scalar(0, 203, 253), lineWidth);
			}
			if (pt3.x != 0 && pt4.x != 0)
			{
				line(outputimage, pt3, pt4, Scalar(67, 238, 245), lineWidth);
			}

			//左臂
			//5-LShoulder
			Point pt5(0, 0);
			if (skeletonPoint[i * 75 + 5 * 3] != 0)
			{
				pt5 = Point((int)skeletonPoint[i * 75 + 5 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 5 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt5, pointWidth, Scalar(0, 253, 184), -1);
			}
			//6-LElbow
			Point pt6(0, 0);
			if (skeletonPoint[i * 75 + 6 * 3] != 0)
			{
				pt6 = Point((int)skeletonPoint[i * 75 + 6 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 6 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt6, pointWidth, Scalar(80, 234, 89), -1);
			}
			//7-LWrist
			Point pt7(0, 0);
			if (skeletonPoint[i * 75 + 7 * 3] != 0)
			{
				pt7 = Point((int)skeletonPoint[i * 75 + 7 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 7 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt7, pointWidth, Scalar(82, 234, 93), -1);
			}
			if (pt1.x != 0 && pt5.x != 0)
			{
				line(outputimage, pt1, pt5, Scalar(0, 253, 184), lineWidth);
			}
			if (pt5.x != 0 && pt6.x != 0)
			{
				line(outputimage, pt5, pt6, Scalar(80, 234, 89), lineWidth);
			}
			if (pt6.x != 0 && pt7.x != 0)
			{
				line(outputimage, pt6, pt7, Scalar(82, 234, 93), lineWidth);
			}

			//8-MidHip
			Point pt8(0, 0);
			if (skeletonPoint[i * 75 + 8 * 3] != 0)
			{
				pt8 = Point((int)skeletonPoint[i * 75 + 8 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 8 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt8, pointWidth, Scalar(31, 42, 244), -1);
			}
			if (pt8.x != 0 && pt1.x != 0)
			{
				line(outputimage, pt8, pt1, Scalar(31, 42, 244), lineWidth);
			}
			//右腿
			//9-RHip
			Point pt9(0, 0);
			if (skeletonPoint[i * 75 + 9 * 3] != 0)
			{
				pt9 = Point((int)skeletonPoint[i * 75 + 9 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 9 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt9, pointWidth, Scalar(76, 252, 30), -1);
			}
			//10-RKnee
			Point pt10(0, 0);
			if (skeletonPoint[i * 75 + 10 * 3] != 0)
			{
				pt10 = Point((int)skeletonPoint[i * 75 + 10 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 10 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt10, pointWidth, Scalar(150, 195, 47), -1);
			}
			//11-RAnkle
			Point pt11(0, 0);
			if (skeletonPoint[i * 75 + 11 * 3] != 0)
			{
				pt11 = Point((int)skeletonPoint[i * 75 + 11 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 11 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt11, pointWidth, Scalar(208, 212, 44), -1);
			}
			if (pt8.x != 0 && pt9.x != 0)
			{
				line(outputimage, pt8, pt9, Scalar(76, 252, 30), lineWidth);
			}
			if (pt9.x != 0 && pt10.x != 0)
			{
				line(outputimage, pt9, pt10, Scalar(150, 195, 47), lineWidth);
			}
			if (pt10.x != 0 && pt11.x != 0)
			{
				line(outputimage, pt10, pt11, Scalar(208, 212, 44), lineWidth);
			}
			//左腿
			//12-RHip
			Point pt12(0, 0);
			if (skeletonPoint[i * 75 + 12 * 3] != 0)
			{
				pt12 = Point((int)skeletonPoint[i * 75 + 12 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 12 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt12, pointWidth, Scalar(193, 150, 53), -1);
			}
			//13-RKnee
			Point pt13(0, 0);
			if (skeletonPoint[i * 75 + 13 * 3] != 0)
			{
				pt13 = Point((int)skeletonPoint[i * 75 + 13 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 13 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt13, pointWidth, Scalar(212, 80, 36), -1);
			}
			//14-RAnkle
			Point pt14(0, 0);
			if (skeletonPoint[i * 75 + 14 * 3] != 0)
			{
				pt14 = Point((int)skeletonPoint[i * 75 + 14 * 3] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 14 * 3 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt14, pointWidth, Scalar(198, 39, 45), -1);
			}
			if (pt8.x != 0 && pt12.x != 0)
			{
				line(outputimage, pt8, pt12, Scalar(193, 150, 53), lineWidth);
			}
			if (pt12.x != 0 && pt13.x != 0)
			{
				line(outputimage, pt12, pt13, Scalar(212, 80, 36), lineWidth);
			}
			if (pt13.x != 0 && pt14.x != 0)
			{
				line(outputimage, pt13, pt14, Scalar(198, 39, 45), lineWidth);
			}
		}
		return outputimage;
	}


	Mat Monitor::InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart)
	{
		Mat img = image;

		for (int i = rowStart; i < logoImage.rows + rowStart && i < img.rows; i++)
			for (int j = colStart; j < logoImage.cols + colStart && j < img.cols; j++)
			{

				float ratio = float(logoImage.at<Vec4b>(i - rowStart, j - colStart)[3]) / 255.f;
				for (int ii = 0; ii < 3; ii++)
				{
					img.at<Vec4b>(i, j)[ii] = uchar(float(logoImage.at<Vec4b>(i - rowStart, j - colStart)[ii]) * ratio
						+ float(img.at<Vec4b>(i, j)[ii]) * (1.f - ratio));
				}
			}

		return img;
	}




	void Monitor::monitorThread()
	{

		std::vector<std::thread*> threads;
		for (auto camera : cameras_)
		{
			auto thread = camera->startGrab();
			threads.push_back(thread);
		}
		auto thread0 = startFusion();
		threads.push_back(thread0);
		auto thread1 = startDetect();
		threads.push_back(thread1);
		auto thread2 = startDisplay();
		threads.push_back(thread2);
		

		for (auto thread : threads)
		{
			thread->join();
			delete thread;
		}
			
	}

	bool Monitor::loadParames()
	{
		// 加载参数到map中;
		std::map<std::string, std::string> parames_map;
		std::string params_file = "./config/VideoSurveillance.config";
		std::string cameras_file = "./config/Cameras.config";


		if (_access(params_file.c_str(), 0) == -1)
		{
			return false;
		}

		if (_access(cameras_file.c_str(), 0) == -1)
		{
			return false;
		}

		tinyxml2::XMLDocument *params_doc = new  tinyxml2::XMLDocument;
		tinyxml2::XMLDocument *cameras_doc = new  tinyxml2::XMLDocument;

		params_doc->LoadFile(params_file.c_str());
		cameras_doc->LoadFile(cameras_file.c_str());
	
		tinyxml2::XMLElement *params_root = params_doc->RootElement();
		tinyxml2::XMLElement *camera_root = cameras_doc->RootElement();

		
		//加载相机参数;
		double mi[9], md[4];
		std::vector<std::string> vct_mi, vct_md;
		cv::Mat intrinsic_matrix, distortion_coeffs;
		for (tinyxml2::XMLElement *camera_xml = camera_root->FirstChildElement(); 
			camera_xml != nullptr; camera_xml = camera_xml->NextSiblingElement())
		{
			shared_ptr<Camera> cameratemp = make_shared<Camera>();
			cameratemp->setID(atoi(camera_xml->Attribute("id")));
			cameratemp->setIP((char *)camera_xml->Attribute("ip"));
			cameratemp->setPort(atoi(camera_xml->Attribute("port")));
			cameratemp->setUser((char*)camera_xml->Attribute("user"));
			cameratemp->setPWD((char *)camera_xml->Attribute("pwd"));
			cameratemp->setSite((char *)camera_xml->Attribute("site"));
			/*!< 加载内参矩阵*/
			std::string mi_str = camera_xml->Attribute("intrinsic_matrix");
			vct_mi = common::split(mi_str, ",");
			assert(vct_mi.size() == 9);
			for (size_t i = 0; i < vct_mi.size(); i++)
			{
				mi[i] = atof(vct_mi[i].c_str());
			}
			intrinsic_matrix = cv::Mat(3, 3, CV_64FC1, mi);
			cameratemp->setIntrinsicMatrix(intrinsic_matrix);

			/*!< 加载畸变矩阵*/
			std::string md_str = camera_xml->Attribute("distortion_coeffs");
			vct_md = common::split(md_str, ",");
			assert(vct_md.size() == 5);
			for (size_t i = 0; i < 4; i++)
			{
				md[i] = atof(vct_md[i].c_str());
			}
			distortion_coeffs = cv::Mat(1, 4, CV_64FC1, md);
			cameratemp->setDistortionCoeffs(distortion_coeffs);
			cameras_.push_back(cameratemp);
		}

		//加载监控参数;
		for (tinyxml2::XMLElement *param_xml = params_root->FirstChildElement(); param_xml != nullptr; param_xml = param_xml->NextSiblingElement())
		{
			parames_map.insert(std::make_pair(param_xml->Attribute("name"), param_xml->Attribute("value")));
		}
		parameValue<bool>(parames_map, "bool", "data_collection_stage", param_.data_collection_stage);
		parameValue<bool>(parames_map, "bool", "image_log_switch", param_.image_log_switch);
		parameValue<bool>(parames_map, "bool", "display_switch", param_.display_switch);
		parameValue<int>(parames_map, "int", "data_from", param_.data_from);
		parameValue<int>(parames_map, "int", "connect_time", param_.connect_time);
		parameValue<int>(parames_map, "int", "reconect_time", param_.reconnect_time);
		parameValue<int>(parames_map, "int", "data_collection_interval", param_.data_collection_interval);
		parameValue<int>(parames_map, "int", "lChannel", param_.lChannel);
		parameValue<int>(parames_map, "int", "dwStreamType", param_.dwStreamType);
		parameValue<int>(parames_map, "int", "dwLinkMode", param_.dwLinkMode);
		parameValue<int>(parames_map, "int", "bBlocked", param_.bBlocked);
		parameValue<int>(parames_map, "int", "image_input_width", param_.image_input_width);
		parameValue<int>(parames_map, "int", "image_input_height", param_.image_input_height);
		parameValue<bool>(parames_map, "bool", "image_input_flip", param_.image_input_flip);
		parameValue<int>(parames_map, "int", "image_input_flipcode", param_.image_input_flipcode);
		parameValue<int>(parames_map, "int", "image_output_width", param_.image_output_width);
		parameValue<int>(parames_map, "int", "image_output_height", param_.image_output_height);
		parameValue<int>(parames_map, "int", "skeleton_desample_rate", param_.skeleton_desample_rate);
		parameValue<int>(parames_map, "int", "object_detect_desample_rate", param_.object_detect_desample_rate);
		delete params_doc;
		delete cameras_doc;

		return true;
	}


	template<class T>
	void Monitor::parameValue(
		std::map<std::string, std::string> &parames_map,
		std::string value_type,
		std::string name,
		T &value)
	{
		/*!< bool */
		if (!strcmp(value_type.c_str(), "bool"))
		{
			if (parames_map.end() != parames_map.find(name))
			{
				std::istringstream(parames_map.find(name)->second) >> std::boolalpha >> value;
			}
		}
		else
		{
			if (parames_map.end() != parames_map.find(name))
			{
				std::istringstream(parames_map.find(name)->second) >> value;
			}
		}
	}
}