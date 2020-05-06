#include "VideoSurveillanceSys/monitor.h"
#include "VideoSurveillanceSys/file_operation.h"
#include "VideoSurveillanceSys/common.h"
#include "tinyxml2/tinyxml2.h"
#include <assert.h>
#include <corecrt_io.h>



using namespace std;

namespace VisionMonitor
{

	Monitor::Monitor()
	{

	}
	Monitor::~Monitor()
	{

	}

	bool Monitor::initiate()
	{
		FileOperation fileopt;

		if (!loadParames())
		{
			return false;
		}

		fileopt.checkAndCreateDir(".\\image_log");

		for (size_t i = 0; i < cameras_.size(); i++)
		{
			
			fileopt.checkAndCreateDir(".\\image_log\\camera" + std::to_string(cameras_[i]->getID()));

			std::cout << "camera[" << cameras_[i]->getID() << "] initialize..." << std::endl;
			if (cameras_[i]->initialize(param_))
			{
				std::cout << "OK" << std::endl;
			}
			else
			{
				std::cout << "Fail" << std::endl;
				return false;
			}

		}


		return true;
	}

	void Monitor::start()
	{
		if (monitorThread_.get_id() == std::thread::id())
		{
			is_start = true;
			monitorThread_ = std::thread(&Monitor::monitorThread, this);
		}
	}

	void Monitor::monitorThread()
	{

		std::vector<std::thread*> threads;
		for (auto camera : cameras_)
		{
			auto thread1 = camera->startGrab();		
			auto thread = camera->startMonitor();	

			threads.push_back(thread1);
			threads.push_back(thread);

		}

		for (auto thread : threads)
		{
			thread->join();
			delete thread;
		}

	}

	bool Monitor::loadParames()
	{
		// 加载参数到map中;
		std::map<std::string, std::string> parames_map;
		std::string params_file = "./config/VideoSurveillance.config";
		std::string cameras_file = "./config/Cameras.config";


		if (_access(params_file.c_str(), 0) == -1)
		{
			return false;
		}

		if (_access(cameras_file.c_str(), 0) == -1)
		{
			return false;
		}

		tinyxml2::XMLDocument *params_doc = new  tinyxml2::XMLDocument;
		tinyxml2::XMLDocument *cameras_doc = new  tinyxml2::XMLDocument;

		params_doc->LoadFile(params_file.c_str());
		cameras_doc->LoadFile(cameras_file.c_str());
	
		tinyxml2::XMLElement *params_root = params_doc->RootElement();
		tinyxml2::XMLElement *camera_root = cameras_doc->RootElement();

		
		//加载相机参数;
		double mi[9], md[4];
		std::vector<std::string> vct_mi, vct_md;
		cv::Mat intrinsic_matrix, distortion_coeffs;
		for (tinyxml2::XMLElement *camera_xml = camera_root->FirstChildElement(); 
			camera_xml != nullptr; camera_xml = camera_xml->NextSiblingElement())
		{
			shared_ptr<Camera> cameratemp = make_shared<Camera>();
			cameratemp->setID(atoi(camera_xml->Attribute("id")));
			cameratemp->setIP((char *)camera_xml->Attribute("ip"));
			cameratemp->setPort(atoi(camera_xml->Attribute("port")));
			cameratemp->setUser((char*)camera_xml->Attribute("user"));
			cameratemp->setPWD((char *)camera_xml->Attribute("pwd"));
			
			/*!< 加载内参矩阵*/
			std::string mi_str = camera_xml->Attribute("intrinsic_matrix");
			vct_mi = common::split(mi_str, ",");
			assert(vct_mi.size() == 9);
			for (size_t i = 0; i < vct_mi.size(); i++)
			{
				mi[i] = atof(vct_mi[i].c_str());
			}
			intrinsic_matrix = cv::Mat(3, 3, CV_64FC1, mi);
			cameratemp->setIntrinsicMatrix(intrinsic_matrix);

			/*!< 加载畸变矩阵*/
			std::string md_str = camera_xml->Attribute("distortion_coeffs");
			vct_md = common::split(md_str, ",");
			assert(vct_md.size() == 5);
			for (size_t i = 0; i < 4; i++)
			{
				md[i] = atof(vct_md[i].c_str());
			}
			distortion_coeffs = cv::Mat(1, 4, CV_64FC1, md);
			cameratemp->setDistortionCoeffs(distortion_coeffs);
			cameras_.push_back(cameratemp);
		}

		//加载监控参数;
		for (tinyxml2::XMLElement *param_xml = params_root->FirstChildElement(); param_xml != nullptr; param_xml = param_xml->NextSiblingElement())
		{
			parames_map.insert(std::make_pair(param_xml->Attribute("name"), param_xml->Attribute("value")));
		}
		parameValue<bool>(parames_map, "bool", "data_collection_stage", param_.data_collection_stage);
		parameValue<bool>(parames_map, "bool", "image_log_switch", param_.image_log_switch);
		parameValue<bool>(parames_map, "bool", "display_switch", param_.display_switch);
		parameValue<int>(parames_map, "int", "data_from", param_.data_from);
		parameValue<int>(parames_map, "int", "connect_time", param_.connect_time);
		parameValue<int>(parames_map, "int", "reconect_time", param_.reconnect_time);
		parameValue<int>(parames_map, "int", "data_collection_interval", param_.data_collection_interval);
		parameValue<int>(parames_map, "int", "lChannel", param_.lChannel);
		parameValue<int>(parames_map, "int", "dwStreamType", param_.dwStreamType);
		parameValue<int>(parames_map, "int", "dwLinkMode", param_.dwLinkMode);
		parameValue<int>(parames_map, "int", "bBlocked", param_.bBlocked);
		parameValue<int>(parames_map, "int", "image_input_width", param_.image_input_width);
		parameValue<int>(parames_map, "int", "image_input_height", param_.image_input_height);
		parameValue<bool>(parames_map, "bool", "image_input_flip", param_.image_input_flip);
		parameValue<int>(parames_map, "int", "image_input_flipcode", param_.image_input_flipcode);
		parameValue<int>(parames_map, "int", "image_output_width", param_.image_output_width);
		parameValue<int>(parames_map, "int", "image_output_height", param_.image_output_height);
		parameValue<int>(parames_map, "int", "skeleton_desample_rate", param_.skeleton_desample_rate);
		parameValue<int>(parames_map, "int", "object_detect_desample_rate", param_.object_detect_desample_rate);
		delete params_doc;
		delete cameras_doc;

		return true;
	}


	template<class T>
	void Monitor::parameValue(
		std::map<std::string, std::string> &parames_map,
		std::string value_type,
		std::string name,
		T &value)
	{
		/*!< bool */
		if (!strcmp(value_type.c_str(), "bool"))
		{
			if (parames_map.end() != parames_map.find(name))
			{
				std::istringstream(parames_map.find(name)->second) >> std::boolalpha >> value;
			}
		}
		else
		{
			if (parames_map.end() != parames_map.find(name))
			{
				std::istringstream(parames_map.find(name)->second) >> value;
			}
		}
	}
}