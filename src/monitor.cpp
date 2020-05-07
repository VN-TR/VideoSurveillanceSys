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

	std::thread* Monitor::startDetect()
	{
		return new std::thread(&Monitor::detect, this);
	}
	void Monitor::detect()
	{
		while (true)
		{
			Timer mytime;
			mytime.tic();

			Mat image = Mat(2300, 3840, CV_8UC3, cvScalar(255, 255, 255));

			for (auto camera : cameras_)
			{
				string site = camera->getSite();
				Mat camera_img = camera->getlastimage();
				if (site == "TL" && camera_img.data != NULL)
				{
					cv::Mat imageROI;
					imageROI = image(cv::Rect(0, 140, 1920, 1080));
					camera_img.copyTo(imageROI);
				}
				if (site == "BL" && camera_img.data != NULL)
				{
					cv::Mat imageROI;
					imageROI = image(cv::Rect(0, 1080+140, 1920, 1080));
					camera_img.copyTo(imageROI);
				}
				if (site == "BR" && camera_img.data != NULL)
				{
					cv::Mat imageROI;
					imageROI = image(cv::Rect(1920, 1080+140, 1920, 1080));
					camera_img.copyTo(imageROI);
				}
			}
			cout << "_time" << mytime.toc() << endl;

			detectThread(image);
			cout << "计算时间" << mytime.toc() << endl;
		
		}
	}

	void Monitor::detectThread(Mat &input)
	{
		if (input.data != NULL)
		{
			detect_time_.tic();
			vector<Saveditem> AI_result;
			Mat display_image;
			AI_result = object_detection_.DL_Detector(input, input, display_image);
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
					skeleton_res = skeleton_estimation(input);
				}
			}
			cout << "骨骼计算:" << detect_time_.toc() << endl;
			Timer displaytime;
			displaytime.tic();
			display(display_image, skeleton_res, AI_result);
			cout << "显示:" << displaytime.toc() << endl;
		}
	}

	void Monitor::display(Mat &object_detect_outimg, vector<float> &skeleton_res, vector<Saveditem> &AI_result)
	{
		if (object_detect_outimg.data != NULL)
		{
			cv::cvtColor(object_detect_outimg, object_detect_outimg, cv::COLOR_BGR2BGRA);
			
			InsertLogo(object_detect_outimg, map_image_, 140, 1920);
			InsertLogo(object_detect_outimg, Title_image_, 0, 0);

			display_image_ = object_detect_outimg;
			if (!skeleton_res.empty())
			{
				Mat skeleton_img = draw_skeleton_image(display_image_, skeleton_res);
				display_image_ = skeleton_img;
			}
			if (!AI_result.empty())
			{
				for (auto res : AI_result)
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

					Point camera_pt(1682, 40);
					Point pts;
					if (abs(x) < 10 && abs(z) < 20)
					{
						pts.x = 1682 + x * 23;
						pts.y = 40 + 475 - z * 23;
						//if (res.itemClass == "Human")
						//{
						//	circle(display_image_, pts, 6, Scalar(0, 0, 255), -1);
						//}
						if (res.itemClass == "Forklift")
						{
							circle(display_image_, pts, 10, Scalar(0, 255, 0), -1);
						}
					}

				}
				filter(skeleton_res, AI_result);
			}

			string camera_id = "camera";
			resize(display_image_, display_image_, Size(param_.image_output_width, param_.image_output_height));
			imshow(camera_id, display_image_);
			waitKey(1);
		}
	}


	void Monitor::filter(vector<float> &skeleton_res, vector<Saveditem> &AI_result)
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
				for (auto res : AI_result)
				{
					if (res.itemClass == "Human")
					{
						float va = (res.itemSite_X1 + res.itemSite_X2) / 2;
						float width = (res.itemSite_X2 - res.itemSite_X1);
						if (abs(pt_human.x - va) < width / 2)
						{
							filter_complete = true;
						}
					}
				}
				if (filter_complete)
				{

					float A = -0.036;
					float B = 9.9;
					float C = 2.45;
					float D = -14.972;

					float kx, ky;
					kx = (pt_human.x - 935.5) / 1164;
					ky = (pt_human.y - 517.8) / 1164;
					float z = -(D) / (A*kx + B * ky + C);
					float x = kx * z;

					Point camera_pt(1682, 40);
					Point pts;
					if (abs(x) < 10 && abs(z) < 20)
					{
						pts.x = 1682 + x * 23;
						pts.y = 40 + 475 - z * 23;
						circle(display_image_, pts, 6, Scalar(0, 0, 255), -1);
					}
				}
			}
		}
	}


	vector<float> Monitor::skeleton_estimation(const Mat input_image)
	{
		Mat skeleton_input_image = input_image;
		resize(skeleton_input_image, skeleton_input_image,
			Size(skeleton_input_image.cols / param_.skeleton_desample_rate,
				skeleton_input_image.rows / param_.skeleton_desample_rate));

		vector<float> skeleton_point;
		auto datumProcessed = opWrapper.emplaceAndPop(skeleton_input_image);
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

			cout << "画图时间" << mytime.toc() << endl;

		}
		return outputimage;
	}


	void Monitor::InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart)
	{
		image_ = image;

		//image_.convertTo(image_, Title_image.type());

		for (int i = rowStart; i < logoImage.rows + rowStart && i < image_.rows; i++)
			for (int j = colStart; j < logoImage.cols + colStart && j < image_.cols; j++)
			{

				float ratio = float(logoImage.at<Vec4b>(i - rowStart, j - colStart)[3]) / 255.f;
				for (int ii = 0; ii < 3; ii++)
				{
					image_.at<Vec4b>(i, j)[ii] = uchar(float(logoImage.at<Vec4b>(i - rowStart, j - colStart)[ii]) * ratio
						+ float(image_.at<Vec4b>(i, j)[ii]) * (1.f - ratio));
				}
			}
	}




	void Monitor::monitorThread()
	{

		std::vector<std::thread*> threads;
		for (auto camera : cameras_)
		{
			auto thread = camera->startGrab();
			threads.push_back(thread);
		}
		auto thread1 = startDetect();
		threads.push_back(thread1);
		
		//
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