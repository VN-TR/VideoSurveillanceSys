/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
 * @file: object_detection.cpp
 * @version: V 1.0.0
 * @author: bcyang@Visionnav.com;
 * @date: 2020-04-19
 * @brief: 物体检测识别
 * @details:
 * @verbatim:
 */

 // INCLUDE
#include "VideoSurveillanceSys/object_detection.h"

// CODE
namespace VisionMonitor
{
	ObjectDetection::ObjectDetection()
	{

	}
	ObjectDetection::~ObjectDetection()
	{

	}
	void ObjectDetection::Init()
	{
		INPUT_NODES_ = { "image_tensor" };
		OUTPUT_NODES_ = { "detection_boxes", "detection_scores", "detection_classes" };
		//_OUTPUT_NODES = {"num_detections" };
		MODEL_NAME_ = "models/object_detection/frozen_inference_graph.pb";
		label_path_ = "models/object_detection/labelmap.pbtxt";
		out_dims_ = { 1 };
		tfutil_ = dnn_tensorflow();
		std::vector<uint8_t> config = { 0x32,0xc,0x9,0x33,0x33, 0x33, 0x33, 0x33, 0x33, 0xc3,0x3f,0x2a,0x1,0x30 };
		readLabelMap();
		tfutil_.InitTFEnvironment(config);
		tfutil_.LoadGraph(MODEL_NAME_.c_str(), true);
		tfutil_.CreateIO_Ops(INPUT_NODES_, input_ops);
		tfutil_.CreateIO_Ops(OUTPUT_NODES_, output_ops);
		tfutil_.BuildSession();
	}
	
	vector<Saveditem> ObjectDetection::DL_Detector(const cv::Mat &img_input, const cv::Mat &draw, cv::Mat &img_output)
	{

		Mat frame = img_input;

		std::vector<unsigned char> input_data;
		std::vector<TF_Tensor*> input_tensor;
		std::vector<TF_Tensor*> output_tensor;

		input_dims_ = { 1, frame.size().height, frame.size().width, frame.channels() };
		input_tensor_element_count_ = input_dims_[0] * input_dims_[1] * input_dims_[2] * input_dims_[3];

		input_data.reserve(input_tensor_element_count_);
		input_data.insert(input_data.end(), (unsigned char*)frame.data,
			(unsigned char*)frame.data + frame.total() * frame.channels());

		input_tensor = { tfutil_.CreateTensor(TF_UINT8,input_dims_,input_data) };
		output_tensor = { tfutil_.CreateEmptyTensor(TF_UINT8,out_dims_), tfutil_.CreateEmptyTensor(TF_UINT8,out_dims_),
		 tfutil_.CreateEmptyTensor(TF_UINT8,out_dims_)};

		tfutil_.RunSession(input_ops, input_tensor.data(), input_tensor.size(),
			output_ops, output_tensor.data(), output_tensor.size());

		auto data = tfutil_.GetTensorsData<float>(output_tensor);

		tfutil_.DeleteTensor(input_tensor);
		tfutil_.DeleteTensor(output_tensor);
		input_data.clear();
		itemInfomation_.clear();

		vector<float> boxes = data[0];
		vector<float> scores = data[1];
		vector<float> classes = data[2];

		int goodindex = 0;

		for (int i = 0; i < scores.size(); i++)
		{
			if (scores[i] > 0.98)
				goodindex++;
		}
		img_output = draw;

		for (int i = 0; i < goodindex; i++)
		{
			string label;
			for (auto lab : label_map_)
			{
				if (classes[i] == lab.id)
					label = lab.labelname;
			}

			cv::Point tl, br;
			tl = cv::Point((int)(boxes[4 * i + 1] * img_output.cols), (int)(boxes[4 * i] * img_output.rows));
			br = cv::Point((int)(boxes[4 * i + 3] * img_output.cols), (int)(boxes[4 * i + 2] * img_output.rows));
			cv::Point mid;
			mid = cv::Point((int)((tl.x + br.x) / 2), (int)((tl.y + br.y) / 2));
			//左上
			if (mid.x < 1920 && mid.y < 1080)
			{
				if (br.x > 1920)
				{
					br = cv::Point(1919, br.y);
				}
				if (br.y > 1080)
				{
					br = cv::Point(br.x, 1079);
				}
			}
			//左下
			else if (mid.x < 1920 && mid.y >1080)
			{
				if (br.x > 1920)
				{
					br = cv::Point(1919, br.y);
				}
				if (tl.y < 1080)
				{
					tl = cv::Point(tl.x,1081);
				}

			}
			//右上
			else if (mid.x > 1920 && mid.y < 1080)
			{
				if (tl.x < 1920)
				{
					br = cv::Point(1921, tl.y);
				}
				if (br.y > 1080)
				{
					br = cv::Point(br.x, 1079);
				}

			}
			//右下
			else if (mid.x > 1920 && mid.y >1080)
			{
				if (tl.x < 1920)
				{
					br = cv::Point(1921, tl.y);
				}		
				if (tl.y < 1080)
				{
					tl = cv::Point(tl.x, 1081);
				}
			}
			itemInfomation_.push_back(Saveditem(label, tl.x, br.x, tl.y, br.y,scores[i]));
		}

		boxes.clear();
		scores.clear();
		classes.clear();
		return itemInfomation_;
	}

	void ObjectDetection::readLabelMap()
	{
		label_map_.resize(5);
		// 初始化tensorflow session
		char buffer[256];
		char id_char[4] = "id:";
		string out_label;
		fstream out;
		out.open(label_path_, ios::in);
		cout << "label_path" << " 的内容如下:" << endl;
		int label_count = 0;
		while (!out.eof())
		{
			out.getline(buffer, 256, '\n');//getline(char *,int,char) 表示该行字符达到256个或遇到换行就结束
			//cout << buffer << endl;
			if (strstr(buffer, "id:"))
			{
				string aa = buffer;
				int bb = std::stoi(aa.substr(5, aa.size() - 1));
				label_map_[label_count].id = bb;
			}
			if (strstr(buffer, "name:"))
			{
				if (strstr(buffer, "Goods"))
				{
					out_label = "Goods";
				}
				else if (strstr(buffer, "Forklift"))
				{
					out_label = "Forklift";
				}
				else if (strstr(buffer, "HandleForklift"))
				{
					out_label = "HandleForklift";
				}
				else if (strstr(buffer, "Human"))
				{
					out_label = "Human";
				}
				else if (strstr(buffer, "Sun"))
				{
					out_label = "Sun";
				}
				label_map_[label_count].labelname = out_label;
				cout << label_map_[label_count].labelname << endl;
				label_count++;
			}

		}
		out.close();
		cout << "load  " + std::to_string(label_count) + " labelmap" << endl;
	}

	void ObjectDetection::Release()
	{
		tfutil_.CleanTFEnv();
	}

}