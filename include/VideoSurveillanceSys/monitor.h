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

		/*!
		* @ brief  初始化
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     bool  返回操作结果
		* @ note	加载参数，初始化所的监测设备
		*/
		bool initiate();
	
		/*!
		* @ brief  开始监测
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     void  
		* @ note
		*/
		void start();

		/*!
		* @ brief  监测线程
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     void  
		* @ note
		*/
		void monitorThread();

		/*!
		* @ brief  开始监测计算线程
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     std::thread*  
		* @ note	
		*/
		std::thread* Monitor::startDetect();

		/*!
		* @ brief  计算线程（循环）
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     void  
		* @ note	
		*/
		void Monitor::detectThread();


		/*!
		* @ brief  显示线程开启
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     std::thread*  
		* @ note
		*/
		std::thread* Monitor::startDisplay();

		/*!
		* @ brief  显示循环
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     void  
		* @ note
		*/
		void Monitor::displayThread();

		/*!
		* @ brief  相机抓图多图合并线程开启
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     std::thread*  
		* @ note
		*/
		std::thread* Monitor::startFusion();

		/*!
		* @ brief  多图融合循环
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     void  
		* @ note
		*/
		void Monitor::fusionThread();

		/*!
		* @ brief  获取最新一帧的图像
		* @ author ybc
		* @ date   2020年5月18日
		* @ return     cv::Mat  最新一帧的图像
		* @ note
		*/
		Mat getlastimage();




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

		/*!
		* @ brief  监测计算
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  Mat & input 原始图像
		* @ param[in]  Mat & AI_input	物体检测降采样输入图
		* @ param[in]  Mat & Ske_input	骨骼检测降采样输入图
		* @ return     void  
		* @ note 包含骨骼监测和物体对象监测计算
		*/
		void detect(Mat &input, Mat &AI_input, Mat &Ske_input);

		/*!
		* @ brief  显示
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const Mat & object_detect_outimg		物体检测输出图
		* @ param[in]  const vector<float> & skeleton_res	骨骼检测结果
		* @ param[in]  const vector<Saveditem> & AI_result	物体识别结果
		* @ return     void  
		* @ note	将计算结果、地图等信息显示
		*/
		void display(const Mat &object_detect_outimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);

		/*!
		* @ brief  滤波（过滤掉框外的骨骼点）
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const vector<float> & skeleton_res	输入的骨骼数据
		* @ param[in]  const vector<Saveditem> & AI_result	输入的物体检测数据
		* @ return     std::vector<float>					输出过滤后的骨骼点
		* @ note	
		*/
		vector<float> filter(const vector<float> &skeleton_res,const vector<Saveditem> &AI_result);

		/*!
		* @ brief  将检测结果定位到地图上显示
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const Mat & displayimg	输入原始图像
		* @ param[in]  const vector<float> & skeleton_res	骨骼数据
		* @ param[in]  const vector<Saveditem> & AI_result	物体检测数据
		* @ return     cv::Mat  包含地图信息的图
		* @ note
		*/
		Mat drawmap(const Mat &displayimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);

		/*!
		* @ brief  对人体骨骼进行评估
		* @ author ybc
		* @ date   2020年5月6日
		* @ param[in]  const Mat input_image	骨骼计算输入图
		* @ return     std::vector<float>		骨骼点数据
		* @ note	基于caffe，在GPU下运算。但是由于openpose特性，使得计算时，暂停其他线程。
		*/
		vector<float> skeleton_estimation(const Mat input_image);

		Mat Monitor::draw_object_detection_image(const Mat input_image, const vector<Saveditem> &AI_result);

		/*!
		* @ brief  根据骨骼点数据画骨骼图像
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const Mat input_image				底图
		* @ param[in]  const vector<float> skeletonPoint	骨骼点
		* @ return     cv::Mat  画骨骼图像
		* @ note
		*/
		Mat draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint);

		/*!
		* @ brief  通过像素定位全局坐标（后视）
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const float & va	图像横坐标
		* @ param[in]  const float & vb	图像纵坐标
		* @ param[out] float & x		全局坐标系横坐标
		* @ param[out] float & z		全局坐标系纵坐标
		* @ return     void  
		* @ note	通过标定地面在相机坐标系中的位置
		*/
		void Monitor::locationPt(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  通过像素定位全局坐标（前视）
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const float & va	图像横坐标
		* @ param[in]  const float & vb	图像纵坐标
		* @ param[out] float & x		全局坐标系横坐标
		* @ param[out] float & z		全局坐标系纵坐标
		* @ return     void  
		* @ note	通过标定地面在相机坐标系中的位置
		*/
		void Monitor::locationPtFront(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  通过像素定位全局坐标（左前）
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const float & va	图像横坐标
		* @ param[in]  const float & vb	图像纵坐标
		* @ param[out] float & x		全局坐标系横坐标
		* @ param[out] float & z		全局坐标系纵坐标
		* @ return     void
		* @ note	通过标定地面在相机坐标系中的位置
		*/
		void Monitor::locationPtFrontLeft(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  通过像素定位全局坐标（右前）
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  const float & va	图像横坐标
		* @ param[in]  const float & vb	图像纵坐标
		* @ param[out] float & x		全局坐标系横坐标
		* @ param[out] float & z		全局坐标系纵坐标
		* @ return     void
		* @ note	通过标定地面在相机坐标系中的位置
		*/
		void Monitor::locationPtFrontRight(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  构建输入大图
		* @ author ybc
		* @ date   2020年5月7日
		* @ param[in]  Mat & input_img
		* @ return     void  
		* @ note 
		*/
		void construct_input_img(Mat &input_img);

		/*!
		* @ brief  在image中插入一张logoImage
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  Mat image		原始大图
		* @ param[in]  Mat logoImage	要插入的小图
		* @ param[in]  int rowStart		插入起点行数
		* @ param[in]  int colStart		插入起点列数
		* @ return     cv::Mat			插入后的图片
		* @ note  这里以四通道的方式插入（支持透明）
		*/
		Mat InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);

		/*!
		* @ brief  在image中插入一张logoImage
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  Mat image		原始大图
		* @ param[in]  Mat logoImage	要插入的小图
		* @ param[in]  int rowStart		插入起点行数
		* @ param[in]  int colStart		插入起点列数
		* @ return     cv::Mat			插入后的图片
		* @ note  这里以三通道的方式插入（不支持透明）
		*/
		Mat InsertLogoJPG(Mat image, Mat logoImage, int rowStart, int colStart);

		/*!
		* @ brief  在image中插入一张logoImage
		* @ author ybc
		* @ date   2020年5月18日
		* @ param[in]  Mat image		原始大图
		* @ param[in]  Mat logoImage	要插入的小图
		* @ param[in]  int rowMid		插入中心点行数
		* @ param[in]  int colMid		插入中心点列数
		* @ param[in]  int site			图片的位置（0代表后视、1代表前视）
		* @ return     cv::Mat			插入后的图片
		* @ note 这里以四通道的方式插入
		*/
		Mat InsertLogoMid(Mat image, Mat logoImage, int rowMid, int colMid , int site);



		private:

		//检测时间
		Timer						total_detect_time_;			/*! <总检测时间 */
		Timer						detect_time_;				/*! <物体检测时间 */
		Timer						skeleton_time_;				/*! <骨骼检测时间 */
		Timer						display_time_;				/*! <图片显示时间 */

		//检测相关
		ObjectDetection				object_detection_;			/*! <物体检测对象 */
		std::vector<Saveditem>      AI_result_;					/*! <物体检测结果 */
		std::vector<float>			skeleton_point_;			/*! <骨骼检测结果 */
		int							skeleton_people_count_;		/*! <骨骼检测人数 */


		//配置参数
		Params						param_;						/*!< 处理参数 */
		std::vector<std::string>    test_image_path_;			/*! <离线数据位置 */

		//标志位
		bool						is_start;					/*!< 监测已开启 */
		bool						last_have_human_;			/*!< 上一帧有人 */

		//运行参数
		int							frame_count_;				/*! <帧数 */
		cv::Mat						image_;						/*! <处理图像 */
		cv::Mat						display_image_;				/*! <显示图像 */
		cv::Mat						Title_image_;				/*! <界面图像 */
		cv::Mat						goods_image_;				/*! <地图货物图标 */
		cv::Mat						forklift_image_;			/*! <地图叉车图标 */

		//线程处理
		std::list<Mat>		        msgRecvQueueMat_;			/*! <相机捕获的图像队列 */
		std::mutex					image_mutex_;				/*! <输入图片锁 */
		std::list<Mat>		        msgRecvQueue_Cal_AI_Mat_;	/*! <物体检测输入图像队列 */
		std::mutex					Cal_AI_image_mutex_;		/*! <物体检测输入图像锁 */
		std::list<Mat>		        msgRecvQueue_Cal_Ske_Mat_;	/*! <骨骼检测输入图像队列 */
		std::mutex					Cal_Ske_image_mutex_;		/*! <骨骼检测输入图像锁 */
		std::list<Mat>		        msgRecvQueue_AI_Mat_;		/*! <物体检测输出图像队列 */
		std::mutex					AI_image_mutex_;			/*! <物体检测输出图像锁 */
		std::list<vector<Saveditem>>msgRecvQueue_AI_Res_;		/*! <物体检测结果队列 */
		std::mutex					AI_Res_mutex_;				/*! <物体检测结果锁 */
		std::list<vector<float>>	msgRecvQueue_Skele_Res_;	/*! <骨骼检测结果队列 */
		std::mutex					Skele_Res_mutex_;			/*! <骨骼检测结果锁 */
		std::list<float>			msgRecvQueue_time_;			/*! <计算速度统计队列 */
		std::mutex					time_mutex_;				/*! <计算速度统计锁 */

		std::vector<std::shared_ptr<Camera>>cameras_;			/*!< 相机列表 */
		std::thread					monitorThread_;				/*!< 监测进程 */
	};


}
#endif	// __MONITOR_HH__
