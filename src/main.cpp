#include <stdio.h>
#include <iostream>
#include "Windows.h"
//#include "HCNetSDK/HCNetSDK.h"
//#include "HCNetSDK/plaympeg4.h"
//#include <time.h>
//#include <openpose/flags.hpp>
//#include <openpose/headers.hpp>
//#include <regex>
//#include <numeric>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include "VideoSurveillanceSys/dnn_tensorflow.hpp"
//#include "VideoSurveillanceSys/timer.h"
//#include <fstream>
//
//using namespace std;
//using namespace cv;
//typedef HWND(WINAPI *PROCGETCONSOLEWINDOW)();
//LONG lPort; //全局的播放库port 号
//HWND hWnd;
//
//struct labelmap
//{
//	int id;
//	string labelname;
//};
//
//void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
//{
//	try
//	{
//		if (datumsPtr != nullptr && !datumsPtr->empty())
//		{
//			Mat skeleten = datumsPtr->at(0)->cvOutputData;
//			resize(skeleten, skeleten, Size(skeleten.cols, skeleten.rows));
//			// Display image
//			cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", skeleten);
//			cv::waitKey(1);
//		}
//		else
//			op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
//	}
//	catch (const std::exception& e)
//	{
//		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//	}
//}
//
//void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
//{
//	try
//	{
//		// Example: How to use the pose keypoints
//		if (datumsPtr != nullptr && !datumsPtr->empty())
//		{
//			// Alternative 1
//			op::log("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString());
//			cout << datumsPtr->at(0)->poseKeypoints.getSize(0) << endl;
//		}
//		else
//			op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
//	}
//	catch (const std::exception& e)
//	{
//		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//	}
//}
//
//
//void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* dwUser)
//{
//	hWnd = GetConsoleWindow();
//	switch (dwDataType)
//	{
//	case NET_DVR_SYSHEAD: //系统头
//		if (!PlayM4_GetPort(&lPort)) //获取播放库未使用的通道号
//		{
//			break;
//		}
//		//m_iPort = lPort;
//		//第一次回调的是系统头，将获取的播放库port 号赋值给全局port，下次回调数据时即使用此port 号播放
//		if (dwBufSize > 0)
//		{
//			if (!PlayM4_SetStreamOpenMode(lPort, STREAME_REALTIME)) //设置实时流播放模式
//			{
//				break;
//			}
//			if (!PlayM4_OpenStream(lPort, pBuffer, dwBufSize, 1024 * 1024)) //打开流接口
//			{
//				break;
//			}
//
//			if (!PlayM4_Play(lPort, hWnd)) //播放开始
//			{
//				break;
//			}
//		}
//		//break;
//	case NET_DVR_STREAMDATA: //码流数据
//		if (dwBufSize > 0 && lPort != -1)
//		{
//			if (!PlayM4_InputData(lPort, pBuffer, dwBufSize))
//			{
//				break;
//			}
//		}
//		break;
//	default: //其他数据
//		if (dwBufSize > 0 && lPort != -1)
//		{
//			if (!PlayM4_InputData(lPort, pBuffer, dwBufSize))
//			{
//				break;
//			}
//		}
//		break;
//	}
//}
//void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
//{
//	char tempbuf[256] = { 0 };
//	switch (dwType)
//	{
//	case EXCEPTION_RECONNECT: //预览时重连
//		printf("----------reconnect--------%d\n", time(NULL));
//		break;
//	default:
//		break;
//	}
//}
//void main() {
//
//	Timer mytime;
//
//	dnn_tensorflow tfutil;
//	std::vector<const char*> INPUT_NODES_;
//	std::vector<const char*> _OUTPUT_NODES;
//	string MODEL_NAME;
//	int INPUT_H;
//	int INPUT_W;
//
//	std::vector<std::int64_t> input_dims;
//	size_t input_tensor_element_count;
//	std::vector<std::int64_t> out_dims;
//	TF_Output input_ops[1];
//	TF_Output output_ops[16];
//	INPUT_NODES_ = { "image_tensor" };
//	_OUTPUT_NODES = { "detection_boxes", "detection_scores", "detection_classes", "num_detections" };
//	//_OUTPUT_NODES = {"num_detections" };
//	MODEL_NAME = "model/frozen_inference_graph.pb";
//	const std::string label_path = "model/labelmap.pbtxt";
//
//
//	vector<labelmap> label_map_;
//	label_map_.resize(5);
//	// 初始化tensorflow session
//	char buffer[256];
//	char id_char[4] = "id:";
//	string out_label;
//	fstream out;
//	out.open(label_path, ios::in);
//	cout << "label_path" << " 的内容如下:" << endl;
//	int label_count = 0;
//	while (!out.eof())
//	{
//		out.getline(buffer, 256, '\n');//getline(char *,int,char) 表示该行字符达到256个或遇到换行就结束
//		//cout << buffer << endl;
//		if (strstr(buffer, "id:"))
//		{
//			string aa = buffer;
//			int bb = std::stoi(aa.substr(5, aa.size() - 1));
//			label_map_[label_count].id = bb;
//		}
//		if (strstr(buffer, "name:"))
//		{
//			if (strstr(buffer, "Goods"))
//			{
//				out_label = "Goods";
//			}
//			else if (strstr(buffer, "Forklift"))
//			{
//				out_label = "Forklift";
//			}
//			else if (strstr(buffer, "HandleForklift"))
//			{
//				out_label = "HandleForklift";
//			}
//			else if (strstr(buffer, "Human"))
//			{
//				out_label = "Human";
//			}
//			else if (strstr(buffer, "Sun"))
//			{
//				out_label = "Sun";
//			}
//			label_map_[label_count].labelname = out_label;
//			cout << label_map_[label_count].labelname << endl;
//			label_count++;
//		}
//
//	}
//	out.close();
//	cout << "load  " + std::to_string(label_count) + " labelmap" << endl;
//
//
//	out_dims = { 1 };
//	tfutil = dnn_tensorflow();
//	//input_dims = { 1,INPUT_H,INPUT_W,3 };
//	/*input_tensor_element_count = input_dims[0] * input_dims[1] * input_dims[2] * input_dims[3];*/
//
//	std::vector<uint8_t> config = { 0x32,0xc,0x9,0x0,0x0, 0x0, 0x0, 0x0,0x0, 0xd0,0x3f,0x2a,0x1,0x30 };
//	//std::vector<uint8_t> config = {};
//	tfutil.InitTFEnvironment(config);
//
//	tfutil.LoadGraph(MODEL_NAME.c_str(), true);
//
//	tfutil.CreateIO_Ops(INPUT_NODES_, input_ops);
//	tfutil.CreateIO_Ops(_OUTPUT_NODES, output_ops);
//
//	tfutil.BuildSession();
//
//
//
//
//	op::log("Starting OpenPose demo...", op::Priority::High);
//
//	// Configuring OpenPose
//	op::log("Configuring OpenPose...", op::Priority::High);
//	op::Wrapper opWrapper{ op::ThreadManagerMode::Asynchronous };
//
//	// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
//	if (FLAGS_disable_multi_thread)
//		opWrapper.disableMultiThreading();
//
//	// Starting OpenPose
//	op::log("Starting thread(s)...", op::Priority::High);
//	opWrapper.start();
//
//	//---------------------------------------
//	// 初始化
//	NET_DVR_Init();
//	//设置连接时间与重连时间
//	NET_DVR_SetConnectTime(2000, 1);
//	NET_DVR_SetReconnect(10000, true);
//
//
//
//	ifstream ifs;
//
//	ifs.open("ip.txt", ios::in);
//	string s;
//	while (ifs >> s) {
//		cout << s << endl;
//	}
//
//	ifs.close();
//	char * ip = &s[0];
//	//---------------------------------------
//	//// 获取控制台窗口句柄
//	//HMODULE hKernel32 = GetModuleHandle("kernel32");
//	//GetConsoleWindow1 = (PROCGETCONSOLEWINDOW)GetProcAddress(hKernel32, "GetConsoleWindow");
//	//---------------------------------------
//	// 注册设备
//	LONG lUserID;
//	NET_DVR_DEVICEINFO_V30 struDeviceInfo;
//	lUserID = NET_DVR_Login_V30(ip, 8000, "admin", "1234qwer", &struDeviceInfo);
//	if (lUserID < 0)
//	{
//		printf("Login error, %d\n", NET_DVR_GetLastError());
//		NET_DVR_Cleanup();
//		return;
//	}
//	//---------------------------------------
//	//设置异常消息回调函数
//	NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);
//	//---------------------------------------
//	//启动预览并设置回调数据流
//	LONG lRealPlayHandle;
//	hWnd = GetConsoleWindow();
//	NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
//	struPlayInfo.hPlayWnd = hWnd; //需要SDK 解码时句柄设为有效值，仅取流不解码时可设为空
//	struPlayInfo.lChannel = 1; //预览通道号
//	struPlayInfo.dwStreamType = 0; //0-主码流，1-子码流，2-码流3，3-码流4，以此类推
//	struPlayInfo.dwLinkMode = 0; //0- TCP 方式，1- UDP 方式，2- 多播方式，3- RTP 方式，4-RTP/RTSP，5-RSTP/HTTP
//	struPlayInfo.bBlocked = 0; //0- 非阻塞取流，1- 阻塞取流
//
//
//	lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, NULL, NULL);
//
//
//
//	if (lRealPlayHandle < 0)
//	{
//		printf("NET_DVR_RealPlay_V40 error\n");
//		NET_DVR_Logout(lUserID);
//		NET_DVR_Cleanup();
//		return;
//	}
//	int i = 0;
//
//
//
//	while (true)
//	{
//		BYTE *pBuffer1 = new BYTE[3000 * 2000];
//		DWORD dwBufSize1 = 1920 * 1080 * 1.5;
//		DWORD dwBufSize2;
//
//		if (PlayM4_GetJPEG(lRealPlayHandle, pBuffer1, dwBufSize1, &dwBufSize2))
//		{
//			cv::Mat rawData(1, dwBufSize2, CV_8UC1, (void*)pBuffer1);
//			cv::Mat decodedImage = cv::imdecode(rawData, 1);
//			if (decodedImage.data != NULL)
//			{
//				cv::resize(decodedImage, decodedImage, cv::Size(960, 540));
//
//				const Mat frame = decodedImage;
//				Mat frame1 = decodedImage;
//				Mat frame2 = decodedImage;
//				const Mat rowframe = decodedImage;
//				//Mat des;
//				//des.create(720, 1280, frame.type());
//				//Mat r1 = des(Rect(0, 0, 640, 360));
//				//frame.copyTo(r1);
//				//Mat r2 = des(Rect(0, 360, 640, 360));
//				//frame1.copyTo(r2);
//				//Mat r3 = des(Rect(640, 0, 640, 360));
//				//frame2.copyTo(r3);
//
//				//frame = des;
//
//
//				std::vector<unsigned char> input_data;
//				std::vector<TF_Tensor*> input_tensor;
//				std::vector<TF_Tensor*> output_tensor;
//
//				input_dims = { 1, frame.size().height, frame.size().width, frame.channels() };
//				input_tensor_element_count = input_dims[0] * input_dims[1] * input_dims[2] * input_dims[3];
//
//				input_data.reserve(input_tensor_element_count);
//				input_data.insert(input_data.end(), (unsigned char*)frame.data,
//					(unsigned char*)frame.data + frame.total() * frame.channels());
//
//				input_tensor = { tfutil.CreateTensor(TF_UINT8,input_dims,input_data) };
//				output_tensor = { tfutil.CreateEmptyTensor(TF_UINT8,out_dims), tfutil.CreateEmptyTensor(TF_UINT8,out_dims),
//				 tfutil.CreateEmptyTensor(TF_UINT8,out_dims), tfutil.CreateEmptyTensor(TF_UINT8,out_dims) };
//
//
//				tfutil.RunSession(input_ops, input_tensor.data(), input_tensor.size(),
//					output_ops, output_tensor.data(), output_tensor.size());
//
//				auto data = tfutil.GetTensorsData<float>(output_tensor);
//
//				vector<float> boxes = data[0];
//				vector<float> scores = data[1];
//				vector<float> classes = data[2];
//
//				int goodindex = 0;
//
//				for (int i = 0; i < scores.size(); i++)
//				{
//					if (scores[i] > 0.8)
//						goodindex++;
//				}
//
//				Mat img = rowframe;
//				for (int i = 0; i < goodindex; i++)
//				{
//					string label;
//					for (auto lab : label_map_)
//					{
//						if (classes[i] == lab.id)
//							label = lab.labelname;
//					}
//
//					cv::Point tl, br;
//					tl = cv::Point((int)(boxes[4 * i + 1] * img.cols), (int)(boxes[4 * i] * img.rows));
//					br = cv::Point((int)(boxes[4 * i + 3] * img.cols), (int)(boxes[4 * i + 2] * img.rows));
//					if (label == "Goods")
//					{
//						cv::rectangle(img, tl, br, cv::Scalar(0, 255, 0), 6);
//					}
//					else if (label == "Human")
//					{
//						cv::rectangle(img, tl, br, cv::Scalar(255, 255, 255), 6);
//					}
//					else if (label == "Sun")
//					{
//						cv::rectangle(img, tl, br, cv::Scalar(230, 220, 150), 6);
//					}
//					else
//					{
//						cv::rectangle(img, tl, br, cv::Scalar(0, 255, 255), 6);
//					}
//
//
//					// Ceiling the score down to 3 decimals (weird!)
//					float scoreRounded = floorf(scores[i] * 1000) / 10;
//					string scoreString = to_string(scoreRounded).substr(0, 4) + "%";
//					string caption = label + " (" + scoreString + ")";
//
//					// Adding caption of type "LABEL (X.XXX)" to the top-left corner of the bounding box
//					int fontCoeff = 25;
//					cv::Point brRect = cv::Point(tl.x + caption.length() * fontCoeff / 1.6, tl.y + fontCoeff);
//
//					cv::Point textCorner = cv::Point(tl.x, tl.y + fontCoeff * 0.9);
//					if (label == "Goods")
//					{
//						cv::rectangle(img, tl, brRect, cv::Scalar(0, 255, 0), -1);
//						cv::putText(img, caption, textCorner, FONT_ITALIC, 0.8, cv::Scalar(0, 0, 0), 2, 16);
//					}
//					else if (label == "Human")
//					{
//						cv::rectangle(img, tl, brRect, cv::Scalar(255, 255, 255), -1);
//						cv::putText(img, caption, textCorner, FONT_ITALIC, 0.8, cv::Scalar(0, 0, 0), 2, 16);
//					}
//
//					else if (label == "Sun")
//					{
//						cv::rectangle(img, tl, brRect, cv::Scalar(230, 220, 150), -1);
//						cv::putText(img, caption, textCorner, FONT_ITALIC, 0.8, cv::Scalar(0, 0, 0), 2, 16);
//					}
//					else
//					{
//						cv::rectangle(img, tl, brRect, cv::Scalar(0, 255, 255), -1);
//						cv::putText(img, caption, textCorner, FONT_ITALIC, 0.8, cv::Scalar(0, 0, 0), 2, 16);
//					}
//				}
//				resize(img, img, Size(img.cols / 2, img.rows / 2));
//
//
//				auto datumProcessed = opWrapper.emplaceAndPop(rowframe);
//				if (datumProcessed != nullptr)
//				{
//					auto s = datumProcessed->at(0)->poseScores.toString();
//					s.erase(s.find_last_not_of(" "));
//
//					//printKeypoints(datumProcessed);
//					//std::cout << "左脚:" << endl;
//					//std::cout << datumProcessed->at(0)->poseKeypoints[57] << " , "
//					//	<< datumProcessed->at(0)->poseKeypoints[58] << " , "
//					//	<< datumProcessed->at(0)->poseKeypoints[59] << std::endl;
//					//std::cout << "右脚:" << endl;
//					//std::cout << datumProcessed->at(0)->poseKeypoints[69] << " , "
//					//	<< datumProcessed->at(0)->poseKeypoints[70] << " , "
//					//	<< datumProcessed->at(0)->poseKeypoints[71] << std::endl;
//
//					display(datumProcessed);
//				}
//				else
//				{
//					op::log("Image could not be processed.", op::Priority::High);
//					continue;
//				}
//
//				cout << "Totol time" << mytime.toc() << endl;
//
//
//			}
//
//		};
//		cout << *pBuffer1 << endl;
//
//
//		delete[] pBuffer1;
//	}
//
//
//
//	Sleep(1000000);
//	//---------------------------------------
//	//关闭预览
//	NET_DVR_StopRealPlay(lRealPlayHandle);
//	//注销用户
//	NET_DVR_Logout(lUserID);
//	NET_DVR_Cleanup();
//	return;
//}
//
#include "VideoSurveillanceSys/monitor.h"
using namespace VisionMonitor;
int main()
{
	Monitor monitor;
	monitor.initiate();
	getchar();
	return 0;
}