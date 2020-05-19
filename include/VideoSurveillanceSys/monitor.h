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

		/*!
		* @ brief  ��ʼ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     bool  ���ز������
		* @ note	���ز�������ʼ�����ļ���豸
		*/
		bool initiate();
	
		/*!
		* @ brief  ��ʼ���
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     void  
		* @ note
		*/
		void start();

		/*!
		* @ brief  ����߳�
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     void  
		* @ note
		*/
		void monitorThread();

		/*!
		* @ brief  ��ʼ�������߳�
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     std::thread*  
		* @ note	
		*/
		std::thread* Monitor::startDetect();

		/*!
		* @ brief  �����̣߳�ѭ����
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     void  
		* @ note	
		*/
		void Monitor::detectThread();


		/*!
		* @ brief  ��ʾ�߳̿���
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     std::thread*  
		* @ note
		*/
		std::thread* Monitor::startDisplay();

		/*!
		* @ brief  ��ʾѭ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     void  
		* @ note
		*/
		void Monitor::displayThread();

		/*!
		* @ brief  ���ץͼ��ͼ�ϲ��߳̿���
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     std::thread*  
		* @ note
		*/
		std::thread* Monitor::startFusion();

		/*!
		* @ brief  ��ͼ�ں�ѭ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     void  
		* @ note
		*/
		void Monitor::fusionThread();

		/*!
		* @ brief  ��ȡ����һ֡��ͼ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     cv::Mat  ����һ֡��ͼ��
		* @ note
		*/
		Mat getlastimage();




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

		/*!
		* @ brief  ������
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  Mat & input ԭʼͼ��
		* @ param[in]  Mat & AI_input	�����⽵��������ͼ
		* @ param[in]  Mat & Ske_input	������⽵��������ͼ
		* @ return     void  
		* @ note ������������������������
		*/
		void detect(Mat &input, Mat &AI_input, Mat &Ske_input);

		/*!
		* @ brief  ��ʾ
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const Mat & object_detect_outimg		���������ͼ
		* @ param[in]  const vector<float> & skeleton_res	���������
		* @ param[in]  const vector<Saveditem> & AI_result	����ʶ����
		* @ return     void  
		* @ note	������������ͼ����Ϣ��ʾ
		*/
		void display(const Mat &object_detect_outimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);

		/*!
		* @ brief  �˲������˵�����Ĺ����㣩
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const vector<float> & skeleton_res	����Ĺ�������
		* @ param[in]  const vector<Saveditem> & AI_result	���������������
		* @ return     std::vector<float>					������˺�Ĺ�����
		* @ note	
		*/
		vector<float> filter(const vector<float> &skeleton_res,const vector<Saveditem> &AI_result);

		/*!
		* @ brief  ���������λ����ͼ����ʾ
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const Mat & displayimg	����ԭʼͼ��
		* @ param[in]  const vector<float> & skeleton_res	��������
		* @ param[in]  const vector<Saveditem> & AI_result	����������
		* @ return     cv::Mat  ������ͼ��Ϣ��ͼ
		* @ note
		*/
		Mat drawmap(const Mat &displayimg, const vector<float> &skeleton_res, const vector<Saveditem> &AI_result);

		/*!
		* @ brief  �����������������
		* @ author ybc
		* @ date   2020��5��6��
		* @ param[in]  const Mat input_image	������������ͼ
		* @ return     std::vector<float>		����������
		* @ note	����caffe����GPU�����㡣��������openpose���ԣ�ʹ�ü���ʱ����ͣ�����̡߳�
		*/
		vector<float> skeleton_estimation(const Mat input_image);

		Mat Monitor::draw_object_detection_image(const Mat input_image, const vector<Saveditem> &AI_result);

		/*!
		* @ brief  ���ݹ��������ݻ�����ͼ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const Mat input_image				��ͼ
		* @ param[in]  const vector<float> skeletonPoint	������
		* @ return     cv::Mat  ������ͼ��
		* @ note
		*/
		Mat draw_skeleton_image(const Mat input_image, const vector<float> skeletonPoint);

		/*!
		* @ brief  ͨ�����ض�λȫ�����꣨���ӣ�
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const float & va	ͼ�������
		* @ param[in]  const float & vb	ͼ��������
		* @ param[out] float & x		ȫ������ϵ������
		* @ param[out] float & z		ȫ������ϵ������
		* @ return     void  
		* @ note	ͨ���궨�������������ϵ�е�λ��
		*/
		void Monitor::locationPt(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  ͨ�����ض�λȫ�����꣨ǰ�ӣ�
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const float & va	ͼ�������
		* @ param[in]  const float & vb	ͼ��������
		* @ param[out] float & x		ȫ������ϵ������
		* @ param[out] float & z		ȫ������ϵ������
		* @ return     void  
		* @ note	ͨ���궨�������������ϵ�е�λ��
		*/
		void Monitor::locationPtFront(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  ͨ�����ض�λȫ�����꣨��ǰ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const float & va	ͼ�������
		* @ param[in]  const float & vb	ͼ��������
		* @ param[out] float & x		ȫ������ϵ������
		* @ param[out] float & z		ȫ������ϵ������
		* @ return     void
		* @ note	ͨ���궨�������������ϵ�е�λ��
		*/
		void Monitor::locationPtFrontLeft(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  ͨ�����ض�λȫ�����꣨��ǰ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  const float & va	ͼ�������
		* @ param[in]  const float & vb	ͼ��������
		* @ param[out] float & x		ȫ������ϵ������
		* @ param[out] float & z		ȫ������ϵ������
		* @ return     void
		* @ note	ͨ���궨�������������ϵ�е�λ��
		*/
		void Monitor::locationPtFrontRight(const float &va, const float &vb,
			float &x, float &z);

		/*!
		* @ brief  ���������ͼ
		* @ author ybc
		* @ date   2020��5��7��
		* @ param[in]  Mat & input_img
		* @ return     void  
		* @ note 
		*/
		void construct_input_img(Mat &input_img);

		/*!
		* @ brief  ��image�в���һ��logoImage
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  Mat image		ԭʼ��ͼ
		* @ param[in]  Mat logoImage	Ҫ�����Сͼ
		* @ param[in]  int rowStart		�����������
		* @ param[in]  int colStart		�����������
		* @ return     cv::Mat			������ͼƬ
		* @ note  ��������ͨ���ķ�ʽ���루֧��͸����
		*/
		Mat InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);

		/*!
		* @ brief  ��image�в���һ��logoImage
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  Mat image		ԭʼ��ͼ
		* @ param[in]  Mat logoImage	Ҫ�����Сͼ
		* @ param[in]  int rowStart		�����������
		* @ param[in]  int colStart		�����������
		* @ return     cv::Mat			������ͼƬ
		* @ note  ��������ͨ���ķ�ʽ���루��֧��͸����
		*/
		Mat InsertLogoJPG(Mat image, Mat logoImage, int rowStart, int colStart);

		/*!
		* @ brief  ��image�в���һ��logoImage
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  Mat image		ԭʼ��ͼ
		* @ param[in]  Mat logoImage	Ҫ�����Сͼ
		* @ param[in]  int rowMid		�������ĵ�����
		* @ param[in]  int colMid		�������ĵ�����
		* @ param[in]  int site			ͼƬ��λ�ã�0������ӡ�1����ǰ�ӣ�
		* @ return     cv::Mat			������ͼƬ
		* @ note ��������ͨ���ķ�ʽ����
		*/
		Mat InsertLogoMid(Mat image, Mat logoImage, int rowMid, int colMid , int site);



		private:

		//���ʱ��
		Timer						total_detect_time_;			/*! <�ܼ��ʱ�� */
		Timer						detect_time_;				/*! <������ʱ�� */
		Timer						skeleton_time_;				/*! <�������ʱ�� */
		Timer						display_time_;				/*! <ͼƬ��ʾʱ�� */

		//������
		ObjectDetection				object_detection_;			/*! <��������� */
		std::vector<Saveditem>      AI_result_;					/*! <�������� */
		std::vector<float>			skeleton_point_;			/*! <��������� */
		int							skeleton_people_count_;		/*! <����������� */


		//���ò���
		Params						param_;						/*!< ������� */
		std::vector<std::string>    test_image_path_;			/*! <��������λ�� */

		//��־λ
		bool						is_start;					/*!< ����ѿ��� */
		bool						last_have_human_;			/*!< ��һ֡���� */

		//���в���
		int							frame_count_;				/*! <֡�� */
		cv::Mat						image_;						/*! <����ͼ�� */
		cv::Mat						display_image_;				/*! <��ʾͼ�� */
		cv::Mat						Title_image_;				/*! <����ͼ�� */
		cv::Mat						goods_image_;				/*! <��ͼ����ͼ�� */
		cv::Mat						forklift_image_;			/*! <��ͼ�泵ͼ�� */

		//�̴߳���
		std::list<Mat>		        msgRecvQueueMat_;			/*! <��������ͼ����� */
		std::mutex					image_mutex_;				/*! <����ͼƬ�� */
		std::list<Mat>		        msgRecvQueue_Cal_AI_Mat_;	/*! <����������ͼ����� */
		std::mutex					Cal_AI_image_mutex_;		/*! <����������ͼ���� */
		std::list<Mat>		        msgRecvQueue_Cal_Ske_Mat_;	/*! <�����������ͼ����� */
		std::mutex					Cal_Ske_image_mutex_;		/*! <�����������ͼ���� */
		std::list<Mat>		        msgRecvQueue_AI_Mat_;		/*! <���������ͼ����� */
		std::mutex					AI_image_mutex_;			/*! <���������ͼ���� */
		std::list<vector<Saveditem>>msgRecvQueue_AI_Res_;		/*! <������������ */
		std::mutex					AI_Res_mutex_;				/*! <���������� */
		std::list<vector<float>>	msgRecvQueue_Skele_Res_;	/*! <������������� */
		std::mutex					Skele_Res_mutex_;			/*! <����������� */
		std::list<float>			msgRecvQueue_time_;			/*! <�����ٶ�ͳ�ƶ��� */
		std::mutex					time_mutex_;				/*! <�����ٶ�ͳ���� */

		std::vector<std::shared_ptr<Camera>>cameras_;			/*!< ����б� */
		std::thread					monitorThread_;				/*!< ������ */
	};


}
#endif	// __MONITOR_HH__
