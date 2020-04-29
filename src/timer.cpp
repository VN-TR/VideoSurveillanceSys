#include "VideoSurveillanceSys/timer.h"


using namespace std::chrono;
Timer::Timer()
{
	tic();
}


Timer::~Timer()
{
}

void Timer::tic()
{
	start_ = system_clock::now();
}

double Timer::toc()
{
	auto now = system_clock::now();
	auto period = duration_cast<microseconds>(now - start_);
	//start_ = now;
	return period.count() / 1000.0;
}