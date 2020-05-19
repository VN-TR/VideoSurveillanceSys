#include <stdio.h>
#include <iostream>
#include "VideoSurveillanceSys/monitor.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/stitching.hpp>
#include<vector>
#include<iostream>
#include<sstream>
#include<string>
#include<time.h> 
#include "math.h"
#include "VideoSurveillanceSys/timer.h"
using namespace std;
using namespace cv;

using namespace VisionMonitor;
//bool try_use_gpu = true;//是否使用GPU，否
//vector<Mat> imgs;
//string result_name = "dst1.jpg";
//
//int main(int argc, char * argv[])
//{
//
//	Mat img1 = imread("2.jpg");
//	Mat img2 = imread("3.jpg");
//
//	imshow("p1", img1);
//	waitKey(1);
//	imshow("p2", img2);
//	waitKey(1);
//	resize(img1, img1, Size(960, 540));
//	resize(img2, img2, Size(960, 540));
//	if (img1.empty() || img2.empty())
//	{
//		cout << "Can't read image" << endl;
//		return -1;
//	}
//	imgs.clear();
//	imgs.push_back(img1);
//	imgs.push_back(img2);
//
//	Stitcher stitcher = Stitcher::createDefault(try_use_gpu); // 使用stitch函数进行拼接
//	stitcher.estimateTransform(imgs);
//	Stitcher st = stitcher;
//	while (true)
//	{
//	Mat pano;
//	Timer mytime;
//	mytime.tic();
//
//	st.composePanorama(pano);
//	cout << "time" << mytime.toc() << endl;
//	//Stitcher::Status status = stitcher.stitch(imgs, pano);
//	//if (status != Stitcher::OK)
//	//{
//	//	cout << "Can't stitch images, error code = " << int(status) << endl;
//	//	return -1;
//	//}
//	resize(pano, pano, Size(960, 540));
//	imwrite(result_name, pano);
//	Mat pano2 = pano.clone();
//
//	
//	imshow("全景图像", pano);
//	waitKey(1);
//	}
//
//
//	return 0;
//}
//
int main()
{
	{
		Monitor monitor;
		monitor.initiate();
		monitor.start();
	}
	getchar();
	return 0;
}



//#include <stdio.h>
//#include <stdlib.h>
//#include <WinSock2.h>
//#include <iostream>
//#include <string>
//#pragma comment(lib, "ws2_32.lib")
//using namespace std;
//
//int main() {
//	WSADATA wsaData;
//	WSAStartup(MAKEWORD(2, 2), &wsaData);
//
//	char * address_input = "www.1688.com";
//	struct hostent *host = gethostbyname(address_input);
//
//	char *address = inet_ntoa(*(struct in_addr*)host->h_addr_list[0]);
//	cout << address << endl;
//
//	system("pause");
//	return 0;
//}

//int main()
//{
//	float x1 = -1.318;
//	float y1 = -0.726;
//	float z1 = 6.6;
//
//	float x2 = 0.88;
//	float y2 = -0.745;
//	float z2 = 6.6;
//
//	float x3 = 1.1;
//	float y3 = 0.778;
//	float z3 = 2.7;
//
//	float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
//	float B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
//	float C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
//
//
//	float D = -(A * x1 + B * y1 + C * z1);
//	cout << "A" << A << endl;
//
//	cout << "B" << B << endl;
//
//	cout << "C" << C << endl;
//
//	cout << "D" << D << endl;
//
//	//float x4 = 1.22;
//	//float y4 = -0.441;
//	//float z4;
//
//	//z4 = -(A*x4 + B * y4 + D) / C;
//	//cout << z4 << endl;
//
//	while (true)
//	{
//		cout << "请输入像素点" << endl;
//		int va;
//		int vb;
//		cin >> va >> vb;
//		cout << "va=" << va << "vb=" << vb << endl;
//		float kx, ky;
//		kx = (va - 961.5) / 1113;
//		ky = (vb - 524.8) / 1114;
//		float z = -(D) / (A*kx + B * ky + C);
//		float x = kx * z ;
//		float y = ky * z;
//		cout << "x" << x << "   z" << z  << endl;
//		float theta1 = acos(x/sqrt(x*x+z*z));
//		cout << theta1 << endl;
//		float theta2 = theta1 + 0.2526;
//		//float theta2 = theta1 - 0.3867;
//		x = cos(theta2) * sqrt(x*x + z*z);
//		z = sin(theta2) * sqrt(x*x + z*z);
//
//		cout << "x" << x << "   z" << z*0.92  << endl;
//	}
//	getchar();
//	return 0;
//}



