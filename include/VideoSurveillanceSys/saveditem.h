#pragma once
#include <iostream>
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;
class Saveditem
{
public:
	Saveditem(string _itemClass, double _itemSite_X1, double _itemSite_X2, double _itemSite_Y1, double _itemSite_Y2 ,float _itemscore);
	~Saveditem();
	void SaveInfo(string _itemClass, double _itemSite_X1, double _itemSite_X2, double _itemSite_Y1, double _itemSite_Y2);

	string itemClass;
	double itemSite_X1;
	double itemSite_X2;
	double itemSite_Y1;
	double itemSite_Y2;
	float  itemscore;
};