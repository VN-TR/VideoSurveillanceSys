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