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
 * @brief ����࣬���ؼ�ز�������ʼ������豸
 */
namespace VisionMonitor
{
	/**
* @brief ���ģ������Ҫ�Ĳ���
*/

	class SURVEILLANCE_API Monitor
	{
	public:
		Monitor();

		~Monitor();

		/**
  * @brief ���ز�������ʼ�����ļ���豸
  * @return ���ز������
  * @retval true ��ʼ���ɹ�
  * @retval false ��ʼ�����ɹ�
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
		* @ brief  ��������ͼ�ز���
		* @ author ybc
		* @ date   2020��4��20��
		* @ return     bool  
		* @ note
		*/
		bool loadParames();

		/**
		* @brief  ���ز�����ֵ
		* @author admin
		* @date   2020��4��20��
		* @param[out] Params &param	�����ṹ��
		* @param[in]  std::map<std::string std::string> & parames_map  ����map
		* @param[in]  std::string value_type  ��������
		* @param[in]  std::string parame_str  ��������
		*/
		template<class T>
		void parameValue(
			std::map<std::string, std::string> &parames_map,
			std::string value_type,
			std::string name,
			T &value
		);




		bool					is_start;				/*!< ����ѿ��� */
		Params				    param_;					/*!< ������� */
		std::vector<std::shared_ptr<Camera>>	    cameras_;				/*!< ����б� */
		std::thread				monitorThread_;			/*!< ������ */


		void display1(const Mat &object_detect_outimg,const vector<float> &skeleton_res,const vector<Saveditem> &AI_result);


		void display(const Mat &object_detect_outimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);

		void filter1(vector<float> &skeleton_res, vector<Saveditem> &AI_result);

		Mat InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);

		vector<float> filter(const vector<float> &skeleton_res,const vector<Saveditem> &AI_result);

		Mat drawmap(const Mat &displayimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);
		/*!
		* @ brief  �����������������
		* @ author ybc
		* @ date   2020��5��6��
		* @ param[in]  const Mat input_image
		* @ return     std::vector<float>
		* @ note	����caffe����GPU�����㡣��������openpose���ԣ�ʹ�ü���ʱ����ͣ�����̡߳�
		*/
		vector<float> skeleton_estimation(const Mat input_image);


		/*!
		* @ brief  ���ƹ���ʶ��ͼ
		* @ author ybc
		* @ date   2020��4��24��
		* @ param[in]  const Mat input_image
		* @ param[out] Mat & output_image
		* @ return     cv::Mat
		* @ note
		*/
		Mat draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint);


		/*!
		* @ brief  ���������ͼ
		* @ author ybc
		* @ date   2020��5��7��
		* @ param[in]  Mat & input_img
		* @ return     void  
		* @ note 
		*/
		void construct_input_img(Mat &input_img);


		Timer						total_detect_time_;
		Timer						detect_time_;				/*! <������ʱ�� */
		Timer						skeleton_time_;				/*! <�������ʱ�� */
		Timer						display_time_;				/*! <ͼƬ��ʾʱ�� */

		//������
		ObjectDetection				object_detection_;			/*! <��������� */
		std::vector<Saveditem>      AI_result_;					/*! <�������� */
		std::vector<float>			skeleton_point_;			/*! <��������� */
		int							skeleton_people_count_;		/*! <����������� */


		//���ò���

		std::vector<std::string>    test_image_path_;			/*! <��������λ�� */

		//��־λ
		bool                        path_loaded_;
		int							frame_count_;


		//���в���
		int							frame_index_;				/*! <֡�� */
		cv::Mat						image_;						/*! <����ͼ�� */
		cv::Mat						display_image_;				/*! <��ʾͼ�� */
		cv::Mat						skeleton_image_;			/*! <����ͼ�� */
		cv::Mat						Title_image_;				/*! <��ǩͼ�� */
		cv::Mat						map_image_;					/*! <��ͼ */
		//�̴߳���
		std::list<Mat>		        msgRecvQueueMat_;			/*! <��������ͼ����� */
		std::mutex					image_mutex_;				/*! <����ͼƬ�� */
		std::list<Mat>		        msgRecvQueue_Cal_AI_Mat_;	/*! <��������ͼ����� */
		std::mutex					Cal_AI_image_mutex_;		/*! <����ͼƬ�� */
		std::list<Mat>		        msgRecvQueue_Cal_Ske_Mat_;	/*! <��������ͼ����� */
		std::mutex					Cal_Ske_image_mutex_;		/*! <����ͼƬ�� */
		std::list<Mat>		        msgRecvQueue_AI_Mat_;		/*! <��������ͼ����� */
		std::mutex					AI_image_mutex_;			/*! <����ͼƬ�� */
		std::list<vector<Saveditem>>msgRecvQueue_AI_Res_;			/*! <��������ͼ����� */
		std::mutex					AI_Res_mutex_;				/*! <����ͼƬ�� */
		std::list<vector<float>>	msgRecvQueue_Skele_Res_;	/*! <��������ͼ����� */
		std::mutex					Skele_Res_mutex_;			/*! <����ͼƬ�� */
		std::list<float>			msgRecvQueue_time_;	/*! <��������ͼ����� */
		std::mutex					time_mutex_;			/*! <����ͼƬ�� */
	};





}
#endif	// __MONITOR_HH__
