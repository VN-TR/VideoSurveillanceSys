#ifndef __MONITOR_HH__
#define __MONITOR_HH__
/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file: monitor.hh
* @version:
* @author: bcyang@Visionnav.com
* @date:
* @brief:
* @details:
* @verbatim:
*/

// INCLUDE PART
#include "VideoSurveillanceSys/camera.h"
#include "VideoSurveillanceSys/interactive.h"
#include <thread>
#include "VideoSurveillanceSys/object_detection.h"
#include <openpose/headers.hpp>
#include <thread>
#include <opencv/cv.h>
#include "map"

// DECLARATION PART
/**
 * @brief 监控类，加载监控参数，初始化监控设备
 */
namespace VisionMonitor
{
	/**
* @brief 监测模块所需要的参数
*/

	class SURVEILLANCE_API Monitor
	{
	public:
		Monitor();

		~Monitor();

		/**
  * @brief 加载参数，初始化所的监测设备
  * @return 返回操作结果
  * @retval true 初始化成功
  * @retval false 初始化不成功
  */
		bool initiate();

		
		void start();

		void monitorThread();

		std::thread* Monitor::startDetect();

		void Monitor::detectThread();

		void detect(Mat &input, Mat &AI_input, Mat &Ske_input);

		std::thread* Monitor::startDisplay();

		void Monitor::displayThread();

		std::thread* Monitor::startFusion();


		void Monitor::fusionThread();


	private:

		/*!
		* @ brief  加载相机和监控参数
		* @ author ybc
		* @ date   2020年4月20日
		* @ return     bool  
		* @ note
		*/
		bool loadParames();

		/**
		* @brief  加载参数的值
		* @author admin
		* @date   2020年4月20日
		* @param[out] Params &param	参数结构体
		* @param[in]  std::map<std::string std::string> & parames_map  参数map
		* @param[in]  std::string value_type  参数类型
		* @param[in]  std::string parame_str  参数名称
		*/
		template<class T>
		void parameValue(
			std::map<std::string, std::string> &parames_map,
			std::string value_type,
			std::string name,
			T &value
		);




		bool					is_start;				/*!< 监测已开启 */
		Params				    param_;					/*!< 处理参数 */
		std::vector<std::shared_ptr<Camera>>	    cameras_;				/*!< 相机列表 */
		std::thread				monitorThread_;			/*!< 监测进程 */


		void display1(const Mat &object_detect_outimg,const vector<float> &skeleton_res,const vector<Saveditem> &AI_result);


		void display(const Mat &object_detect_outimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);

		void filter1(vector<float> &skeleton_res, vector<Saveditem> &AI_result);

		Mat InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);

		vector<float> filter(const vector<float> &skeleton_res,const vector<Saveditem> &AI_result);

		Mat drawmap(const Mat &displayimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);
		/*!
		* @ brief  对人体骨骼进行评估
		* @ author ybc
		* @ date   2020年5月6日
		* @ param[in]  const Mat input_image
		* @ return     std::vector<float>
		* @ note	基于caffe，在GPU下运算。但是由于openpose特性，使得计算时，暂停其他线程。
		*/
		vector<float> skeleton_estimation(const Mat input_image);


		/*!
		* @ brief  绘制骨骼识别图
		* @ author ybc
		* @ date   2020年4月24日
		* @ param[in]  const Mat input_image
		* @ param[out] Mat & output_image
		* @ return     cv::Mat
		* @ note
		*/
		Mat draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint);


		/*!
		* @ brief  构建输入大图
		* @ author ybc
		* @ date   2020年5月7日
		* @ param[in]  Mat & input_img
		* @ return     void  
		* @ note 
		*/
		void construct_input_img(Mat &input_img);


		Timer						total_detect_time_;
		Timer						detect_time_;				/*! <物体检测时间 */
		Timer						skeleton_time_;				/*! <骨骼检测时间 */
		Timer						display_time_;				/*! <图片显示时间 */

		//检测相关
		ObjectDetection				object_detection_;			/*! <物体检测对象 */
		std::vector<Saveditem>      AI_result_;					/*! <物体检测结果 */
		std::vector<float>			skeleton_point_;			/*! <骨骼检测结果 */
		int							skeleton_people_count_;		/*! <骨骼检测人数 */


		//配置参数

		std::vector<std::string>    test_image_path_;			/*! <离线数据位置 */

		//标志位
		bool                        path_loaded_;
		int							frame_count_;


		//运行参数
		int							frame_index_;				/*! <帧数 */
		cv::Mat						image_;						/*! <处理图像 */
		cv::Mat						display_image_;				/*! <显示图像 */
		cv::Mat						skeleton_image_;			/*! <骨骼图像 */
		cv::Mat						Title_image_;				/*! <标签图像 */
		cv::Mat						map_image_;					/*! <地图 */
		//线程处理
		std::list<Mat>		        msgRecvQueueMat_;			/*! <相机捕获的图像队列 */
		std::mutex					image_mutex_;				/*! <输入图片锁 */
		std::list<Mat>		        msgRecvQueue_Cal_AI_Mat_;	/*! <相机捕获的图像队列 */
		std::mutex					Cal_AI_image_mutex_;		/*! <输入图片锁 */
		std::list<Mat>		        msgRecvQueue_Cal_Ske_Mat_;	/*! <相机捕获的图像队列 */
		std::mutex					Cal_Ske_image_mutex_;		/*! <输入图片锁 */
		std::list<Mat>		        msgRecvQueue_AI_Mat_;		/*! <相机捕获的图像队列 */
		std::mutex					AI_image_mutex_;			/*! <输入图片锁 */
		std::list<vector<Saveditem>>msgRecvQueue_AI_Res_;			/*! <相机捕获的图像队列 */
		std::mutex					AI_Res_mutex_;				/*! <输入图片锁 */
		std::list<vector<float>>	msgRecvQueue_Skele_Res_;	/*! <相机捕获的图像队列 */
		std::mutex					Skele_Res_mutex_;			/*! <输入图片锁 */
		std::list<float>			msgRecvQueue_time_;	/*! <相机捕获的图像队列 */
		std::mutex					time_mutex_;			/*! <输入图片锁 */
	};





}
#endif	// __MONITOR_HH__
