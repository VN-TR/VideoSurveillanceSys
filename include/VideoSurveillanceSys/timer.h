#pragma once
#include <chrono>
class Timer
{
public:
	Timer();
	~Timer();
	void tic();
	double toc();

private:
	std::chrono::system_clock::time_point start_;
};

