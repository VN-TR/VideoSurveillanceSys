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
	char filename[75]; //�������飬���������С���ڵ���ͼƬ�ĸ���
	int isColor = 1;   //���Ϊ0 ��������Ҷ�ͼ��
	int fps = 12;      //���������Ƶ��֡��
	//int frameWidth = image.cols;   //��֡ͼƬ�Ŀ�
	//int frameHeight = image.rows;  //��֡ͼƬ�ĸ�

	VideoWriter writer("video_out.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,
		Size(1920, 1080), isColor);

	for (unsigned int i = 0; i < 1000; i++)
	{
		sprintf(filename, "display/%d.jpg", i);//�ڶ���������ָ��ͼƬ·����ͼƬ����һ��ʽ��image%d��
		image = imread(filename);//����ͼƬ
		if (image.empty())
		{
			break;
		}
		waitKey(0);
		writer.write(image); 
	}
}





