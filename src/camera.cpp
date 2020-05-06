/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
 * @file: camera.cc
 * @version: V 1.0.0
 * @author: xzhang@Visionnav.com;
 * @date: 2018-08-21
 * @brief: 相机类函数实现;
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/camera.h"
#include "VideoSurveillanceSys/monitor.h"
#include "VideoSurveillanceSys/file_operation.h"
#include "VideoSurveillanceSys/common.h"

#include <io.h>
#include <assert.h>
#include <opencv2/opencv.hpp>
#include "VideoSurveillanceSys/timer.h"
using namespace std;
// CODE
namespace VisionMonitor
{
	Camera::Camera() :frame_index_(0), path_loaded_(0){}
	Camera::~Camera() {}
	
	op::Wrapper opWrapper{ op::ThreadManagerMode::Asynchronous };


	bool Camera::initialize(const Params &param)
	{
		param_ = param;

		//如果是不是图片采集阶段（运行阶段）则初始化物体检测和骨骼检测
		if (param_.data_collection_stage == false)
		{
			
			object_detection_.Init();
			opWrapper.start();
			//skeleton_estimation_.init();
			//opWrapper.disableMultiThreading();
			

			Title_image_ = cv::imread("./inform_image/1.png", CV_LOAD_IMAGE_UNCHANGED);
			Inform_car_image_ = cv::imread("./inform_image/2.png", CV_LOAD_IMAGE_UNCHANGED);
			Inform_human_image_ = cv::imread("./inform_image/3.png", CV_LOAD_IMAGE_UNCHANGED);
			Inform_good_image_ = cv::imread("./inform_image/4.png", CV_LOAD_IMAGE_UNCHANGED);
			map_image_ = cv::imread("./inform_image/5.png", CV_LOAD_IMAGE_UNCHANGED);
		}
		//如果是图片采集阶段或运行阶段使用实时数据时,初始化相机硬件
		if (param_.data_collection_stage == true || param_.data_from)
		{
			HKinit(param_);
		}

		return true;
	}

	bool Camera::HKinit(const Params &param)
	{
		NET_DVR_Init();
		//设置连接时间与重连时间
		NET_DVR_SetConnectTime(param.connect_time, 1);
		NET_DVR_SetReconnect(param.reconnect_time, true);

		char ip[128], user[128], pwd[128];
		strcpy(ip, ip_.c_str());
		strcpy(user, user_.c_str());
		strcpy(pwd, pwd_.c_str());
		lUserID_ = NET_DVR_Login_V30(ip, port_, user, pwd, &struDeviceInfo_);
		if (lUserID_ < 0)
		{
			printf("Login error, %d\n", NET_DVR_GetLastError());
			NET_DVR_Cleanup();
			return false;
		}
		//启动预览并设置回调数据流
		hWnd_ = GetConsoleWindow();
		NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
		struPlayInfo.hPlayWnd = hWnd_; //需要SDK 解码时句柄设为有效值，仅取流不解码时可设为空
		struPlayInfo.lChannel = param.lChannel; //预览通道号
		struPlayInfo.dwStreamType = param.dwStreamType; //0-主码流，1-子码流，2-码流3，3-码流4，以此类推
		struPlayInfo.dwLinkMode = param.dwLinkMode; //0- TCP 方式，1- UDP 方式，2- 多播方式，3- RTP 方式，4-RTP/RTSP，5-RSTP/HTTP
		struPlayInfo.bBlocked = param.bBlocked; //0- 非阻塞取流，1- 阻塞取流

		lRealPlayHandle_ = NET_DVR_RealPlay_V40(lUserID_, &struPlayInfo, NULL, NULL);
		return true;
	}

	std::thread* Camera::startGrab()
	{
		return new std::thread(&Camera::grabThread, this);
	}

	void Camera::grabThread()
	{
		while (true)
		{
			grab_time_.tic();
			std::string pic_name;
			Mat grabimg = grabbingFrame(param_, pic_name);
			if (grabimg.data != NULL)
			{
				Mat distortimg;
				cv::undistort(grabimg, distortimg, getIntrinsicMatrix(), getDistortionCoeffs());
				grabimg = distortimg;
				{
					std::lock_guard<std::mutex> locker_image(image_mutex_);
					msgRecvQueueMat_.push_back(grabimg);
					if (msgRecvQueueMat_.size() > 2)
					{
						msgRecvQueueMat_.pop_front();
					}
				}
				
				cout << "camera "<<getID()<< " 抓图时间: " << grab_time_.toc() << "ms"<< endl;
			}
			Sleep(10);
		}
	}


	std::thread* Camera::startMonitor()
	{
		return new std::thread(&Camera::monitorThread, this);
	}


	void Camera::monitorThread()
	{
	
		while (true)
		{
			Mat detect_input_img;
			{
				std::lock_guard<std::mutex> locker_image(image_mutex_);

				if (!msgRecvQueueMat_.empty())
				{
					detect_input_img = msgRecvQueueMat_.back();
				}
			}

			if (detect_input_img.data != NULL)
			{
				detect_time_.tic();
				vector<Saveditem> AI_result;
				Mat display_image;
				AI_result = object_detection_.DL_Detector(detect_input_img, detect_input_img, display_image);
				bool havepeople = false;
				for (auto res : AI_result)
				{
					cout << "res.itemClass" << res.itemClass << endl;
					if (res.itemClass == "Human")
					{
						havepeople = true;
					}
				}
				vector<float> skeleton_res;
				if (havepeople)
				{
					if (detect_input_img.data != NULL)
					{
						//skeleton_time_.tic();
						
						skeleton_res = skeleton_estimation(detect_input_img);
						cout << "camera " << getID() << " 骨骼计算时间: " << detect_time_.toc() << "ms" << endl;
					}
				}
				display(display_image, skeleton_res, AI_result);
			}

			Sleep(10);

		}
	}
	void Camera::filter(vector<float> &skeleton_res, vector<Saveditem> &AI_result)
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

	void Camera::display(Mat &object_detect_outimg,vector<float> &skeleton_res,vector<Saveditem> &AI_result)
	{
		if (object_detect_outimg.data != NULL)
		{
			cv::cvtColor(object_detect_outimg, object_detect_outimg, cv::COLOR_BGR2BGRA);
			InsertLogo(object_detect_outimg, Title_image_, 0, 0);
			InsertLogo(object_detect_outimg, map_image_, 40, 1445);
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

			string camera_id = "camera" + to_string(getID());
			resize(display_image_, display_image_, Size(960, 540));
			imshow(camera_id, display_image_);
			waitKey(1);
		}
	}


	cv::Mat Camera::grabbingFrame(Params &params,  std::string &img_name)
	{
		char PicName[256] = { 0 };
		char sJpegPicBuffer[1024] = { 0 };
		cv::Mat img, org;
		frame_index_++;
		if (0== param_.data_from && !path_loaded_)
		{
			path_loaded_ = true;
			FileOperation               file_opt;
			std::string test_image_dir = "./test_image/camera" + std::to_string(getID());
			cout << test_image_dir << endl;
			file_opt.getFileNameList(test_image_dir, ".jpg", test_image_path_);
		}
		

		cv::Mat org_image;
		// read image
		if (0== param_.data_from)
		{
			if (frame_index_ >= test_image_path_.size()) return img;
			std::string  img_path = "./test_image/camera" + std::to_string(getID()) + "/" + test_image_path_[frame_index_];
			org_image = cv::imread(img_path);
			img = org_image;
		}
		else
		{
			int64_t time = common::get_time_stamp();
			LONG iCurChan = lRealPlayHandle_;
			sprintf_s(PicName, ".\\image_log\\camera%d\\%I64d_ch%02ld.jpg", id_, time, iCurChan);

			BYTE *pBuffer1 = new BYTE[3000 * 2000];
			DWORD dwBufSize1 = 1920 * 1080 * 1.5;
			DWORD dwBufSize2;

			if (PlayM4_GetJPEG(lRealPlayHandle_, pBuffer1, dwBufSize1, &dwBufSize2))
			{
				cv::Mat rawData(1, dwBufSize2, CV_8UC1, (void*)pBuffer1);
				cv::Mat decodedImage = cv::imdecode(rawData, 1);
				if (decodedImage.data != NULL)
				{
					img = decodedImage;
					if (param_.image_input_flip)
					{
						flip(img,img, param_.image_input_flipcode);
					}
					resize(img, img, Size(param_.image_input_width, param_.image_input_height));
					if (params.image_log_switch && !params.data_collection_stage)
					{
						imwrite(PicName, img);
					}
					else if (params.data_collection_stage)
					{
						string cameraid = to_string(id_);
						imshow(cameraid, img);
						imwrite(PicName, img);
						
						waitKey(params.data_collection_interval);
					}
				}
			};
			delete[] pBuffer1;
		}

		return img;
	}

	void Camera::setID(int id)
	{
		id_ = id;
	}

	int Camera::getID(void)
	{
		return id_;
	}

	void Camera::setIP(std::string ip)
	{
		ip_ = ip;
	}

	std::string Camera::getIP(void)
	{
		return ip_;
	}

	void Camera::setPort(int port)
	{
		port_ = port;
	}

	int Camera::getPort(void)
	{
		return port_;
	}

	void Camera::setUser(std::string user)
	{
		user_ = user;
	}

	std::string Camera::getUser(void)
	{
		return user_;
	}


	void Camera::setPWD(std::string pwd)
	{
		pwd_ = pwd;
	}

	std::string Camera::getPWD(void)
	{
		return pwd_;
	}

	cv::Mat	Camera::getIntrinsicMatrix()
	{
		return intrinsic_matrix_;
	}

	void Camera::setIntrinsicMatrix(cv::Mat &intrinsic_matrix)
	{
		intrinsic_matrix.copyTo(intrinsic_matrix_);
	}

	cv::Mat	Camera::getDistortionCoeffs()
	{
		return distortion_coeffs_;
	}

	void Camera::setDistortionCoeffs(cv::Mat &distortion_coeffs)
	{
		distortion_coeffs.copyTo(distortion_coeffs_);
	}

	Mat Camera::getlastimage()
	{
		return display_image_;
	}

	void Camera::drawMap(Mat &inputmat)
	{

	}

	vector<float> Camera::skeleton_estimation(const Mat input_image)
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


	Mat Camera::draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint)
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
			Point pt0(0,0);
			if (skeletonPoint[i * 75] != 0)
			{
				pt0 = Point((int)skeletonPoint[i * 75] * param_.skeleton_desample_rate,
					(int)skeletonPoint[i * 75 + 1] * param_.skeleton_desample_rate);
				circle(outputimage, pt0, pointWidth, Scalar(74, 38, 252), -1);
			}
			//1-Neck
			Point pt1(0, 0);
			if (skeletonPoint[i * 75 + 1*3] != 0)
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


	void Camera::InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart)
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


}