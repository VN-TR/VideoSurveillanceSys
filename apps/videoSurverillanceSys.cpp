#include <stdio.h>
#include <iostream>
#include "VideoSurveillanceSys/monitor.h"
using namespace VisionMonitor;
int main()
{
	float x1 = -0.98;
	float y1 = 0.669;
	float z1 = 3.38;

	float x2 = -0.98;
	float y2 = -0.449;
	float z2 = 7.88;

	float x3 = 1.22;
	float y3 = -0.441;
	float z3 = 7.88;
		
	float A = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
	float B = (x3 - x1)*(z2 - z1) - (x2 - x1)*(z3 - z1);
	float C = (x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1);
	

	float D = -(A * x1 + B * y1 + C * z1);
	cout << "A" << A << endl;

	cout << "B" << B << endl;

	cout << "C" << C << endl;

	cout << "D" << D << endl;

	float x4 = 1.22;
	float y4 = -0.441;
	float z4 ;

	z4 = -(A*x4 + B * y4 + D) / C;
	cout << z4 << endl;

	while (true)
	{
		cout << "ÇëÊäÈëÏñËØµã" << endl;
		int va;
		int vb;
		cin >> va >> vb;
		cout << "va=" << va << "vb=" << vb << endl;
		float kx, ky;
		kx = (va - 935.5) / 1164;
		ky = (vb - 517.8) / 1164;
		float z = -(D) / (A*kx + B * ky + C);
		float x = kx * z;
		cout << "x" << x << "z" << z * 0.89 << endl;
	}

	//Monitor monitor;
	//monitor.initiate();
	//monitor.start();
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