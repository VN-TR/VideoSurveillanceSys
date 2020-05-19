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

int main()
{
	Mat image;
	char filename[75]; //定义数组，其中数组大小大于等于图片的个数
	int isColor = 1;   //如果为0 ，可输出灰度图像
	int fps = 12;      //设置输出视频的帧率
	//int frameWidth = image.cols;   //单帧图片的宽
	//int frameHeight = image.rows;  //单帧图片的高

	VideoWriter writer("video_out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
		Size(1920, 1080), isColor);

	for (unsigned int i = 0; i < 1000; i++)
	{
		sprintf(filename, "display/%d.jpg", i);//第二个参数是指定图片路径和图片名的一般式（image%d）
		image = imread(filename);//导入图片
		if (image.empty())
		{
			break;
		}
		waitKey(0);
		writer.write(image); 
	}
}





