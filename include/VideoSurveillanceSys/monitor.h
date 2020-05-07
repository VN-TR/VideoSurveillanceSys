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

		void Monitor::detect();

		void detectThread(Mat &input);

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


		void display(Mat &object_detect_outimg, vector<float> &skeleton_res, vector<Saveditem> &AI_result);

		void filter(vector<float> &skeleton_res, vector<Saveditem> &AI_result);

		void InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);




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


		//���в���
		int							frame_index_;				/*! <֡�� */
		cv::Mat						image_;						/*! <����ͼ�� */
		cv::Mat						display_image_;				/*! <��ʾͼ�� */
		cv::Mat						skeleton_image_;			/*! <����ͼ�� */
		cv::Mat						Title_image_;				/*! <��ǩͼ�� */
		cv::Mat						Inform_car_image_;			/*! <��ǩͼ�� */
		cv::Mat						Inform_human_image_;		/*! <��ǩͼ�� */
		cv::Mat						Inform_good_image_;			/*! <��ǩͼ�� */
		cv::Mat						map_image_;					/*! <��ͼ */

	};





}
#endif	// __MONITOR_HH__
