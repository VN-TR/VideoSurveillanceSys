#include <stdio.h>
#include <iostream>
#include "VideoSurveillanceSys/monitor.h"
using namespace VisionMonitor;
int main()
{
	Monitor monitor;
	monitor.initiate();
	monitor.start();
	getchar();
	return 0;
}