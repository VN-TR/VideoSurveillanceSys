#pragma once
#include <chrono>
#include "VideoSurveillanceSys/interactive.h"
class SURVEILLANCE_API Timer
{
public:
	Timer();
	~Timer();
	void tic();
	double toc();

private:
	std::chrono::system_clock::time_point start_;
};

