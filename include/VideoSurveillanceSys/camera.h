#pragma once

#ifndef __CAMERA_H__
#define __CAMERA_H__
/**Copyright (C) 2020-2020 Visionnav Robotics Limited. All right reserved
* @file: camera.hh
* @version: V 1.0.0
* @author: bcyang@Visionnav.com;
* @date: 2020-04-18
* @brief: �����ͷ�ļ�;
* @details:
* @verbatim:
*/

// INCLUDE PART
#include <opencv/cv.h>
#include "HCNetSDK/HCNetSDK.h"
#include "HCNetSDK/plaympeg4.h"
#include <thread>
#include <openpose/headers.hpp>
#include "VideoSurveillanceSys/timer.h"
using namespace cv;
typedef HWND(WINAPI *PROCGETCONSOLEWINDOW)();

// DECLARATION PART
namespace VisionMonitor
{

	struct Params
	{
		bool data_collection_stage = false;			/*!< �Ƿ�Ϊ�ɼ����ݽ׶� true��ͼƬ�ɼ� false�����н׶� */
		int data_collection_interval = 2000;		/*!< �ɼ����ݼ�� /���� */
		bool image_log_switch = true;		        /*!< ��¼ͼƬ��־���� */
		bool display_switch = true;		            /*!< ͼ����ʾ������� */
		bool show_FPS = false;						/*!< ��ӡ֡�� */
		bool show_current_time = true;				/*!< ��ӡ��ǰʱ�� */
		bool obtain_video = false;				    /*!< ��ȡ��Ƶ����Ϣ */
		int data_from = 1;							/*!< 1��ʵʱ 0����̬����./test_image */
		int	connect_time = 2000;					/*!< ����ʱ�� */
		int	reconnect_time = 10000;					/*!< ����ʱ�� */
		int lChannel = 1;							/*!< Ԥ��ͨ���� */
		int dwStreamType = 0;						/*!< 0 - ��������1 - ��������2 - ����3��3 - ����4���Դ����� */
		int dwLinkMode = 0;							/*!< 0 - TCP ��ʽ��1 - UDP ��ʽ��2 - �ಥ��ʽ��3 - RTP ��ʽ��4 - RTP / RTSP��5 - RSTP / HTTP */
		int bBlocked = 0;							/*!< 0 - ������ȡ����1 - ����ȡ�� */
		int image_input_width = 1920;				/*!< ����ͼƬ��� */
		int image_input_height = 1080;				/*!< ����ͼƬ�߶� */
		bool image_input_flip = false;				/*!< ����ͼƬ��ת */
		int image_input_flipcode = -1;				/*!< ����ͼƬ��ת���� >0: ��y-�ᷭת, 0: ��x-�ᷭת, <0: x��y��ͬʱ��ת*/
		int image_output_width = 1920;				/*!< ��ʾͼƬ��� */
		int image_output_height = 1080;				/*!< ��ʾͼƬ�߶� */
		int skeleton_desample_rate = 4;				/*!< ����ʶ�𽵲����� */
		int object_detect_desample_rate = 2;		/*!< �����⽵������ */
		int obtain_video_FPS = 7;					/*!< �����Ƶ��FPS */
		int obtain_video_color = 1;					/*!< �����Ƶ����ɫ  0 - �Ҷ� 1 - ��ɫ */
		int obtain_video_width = 1920;				/*!< �����Ƶ����� */
		int obtain_video_height = 1080;				/*!< �����Ƶ���߶� */
		bool only_show_front = false;				/*!< ֻ��ʾǰ�� */
	};


	/**
	 * @brief �����
	 */
	class Camera
	{
	public:

		Camera();

		~Camera();

		/*!
		* @ brief  ��ʼ�����,��������ʱ��������ʱ�䣬ע���豸����ȡ�豸������Ϣ
		* @ author ybc
		* @ date   2020��4��20��
		* @ param[in]  const Params & param �����ز���
		* @ return     bool  ���������ʼ�����
		* @ note
		*/
		bool initialize(const Params &param);

		/*!
		* @ brief  ���Ӳ����ʼ��
		* @ author ybc
		* @ date   2020��4��20��
		* @ param[in]  const Params & param �����ز���
		* @ return     bool  ���������ʼ�����
		* @ note
		*/
		bool HKinit(const Params &param);


		void HKClean();

		/*!
		* @ brief  ץͼ
		* @ author ybc
		* @ date   2020��4��29��
		* @ return     std::thread*  
		* @ note
		*/
		std::thread* startGrab();



		/*!
		* @ brief  ץͼ�߳�
		* @ author ybc
		* @ date   2020��4��29��
		* @ return     void  
		* @ note
		*/
		void grabThread();


		/*!
		* @ brief  ��һ��ͼƬ
		* @ author ybc
		* @ date   2020��4��29��
		* @ param[in]  Params & params	������
		* @ param[in]  std::string & img_name	ͼƬ��
		* @ return     cv::Mat  ���ץ����ͼƬ
		* @ note	�����е�ͼƬ�ĳ���
		*/
		cv::Mat grabbingFrame(Params &params, std::string &img_name);

		/**
		* @brief �����豸���
		* @param[in] int id �豸���
		* @return ��
		* @retval void
		*/
		void setID(int id);

		/**
		  * @brief ��ȡ�豸���
		  * @return �����豸���
		  * @retval int �豸���
		  */
		int getID(void);

		/**
		  * @brief �����豸IP��ַ
		  * @param[in] std::string ip �豸IP��ַ
		  * @return ��
		  * @retval void
		  */
		void setIP(std::string ip);

		/**
		  * @brief ��ȡ�豸IP��ַ
		  * @return �����豸IP��ַ
		  * @retval std::string �豸IP��ַ
		  */
		std::string getIP(void);

		/**
		  * @brief �����豸�˿ں�
		  * @param[in] int port �豸�˿ں�
		  * @return ��
		  * @retval void
		  */
		void setPort(int port);

		/**
		  * @brief ��ȡ�豸���
		  * @return �����豸�˿ں�
		  * @retval int �豸�˿ں�
		  */
		int getPort(void);

		/**
		  * @brief �����豸���
		  * @param[in] std::string user �豸ע���û���
		  * @return ��
		  * @retval void
		  */
		void setUser(std::string user);

		/**
		  * @brief ��ȡ�豸ע���û���
		  * @return �����豸ע���û���
		  * @retval std::string �豸ע���û���
		  */
		std::string getUser(void);

		/**
		  * @brief �����豸ע������
		  * @param[in] std::string pwd �豸ע������
		  * @return ��
		  * @retval void
		  */
		void setPWD(std::string pwd);

		/**
		  * @brief ��ȡ�豸ע������
		  * @return �����豸ע������
		  * @retval std::string �豸ע������
		  */
		std::string getPWD(void);


		/*!
		* @ brief  ��������ںϲ�ͼ�е�λ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ param[in]  std::string pwd
		* @ return     void  
		* @ note
		*/
		void setSite(std::string pwd);
	

		/*!
		* @ brief  ��ȡ����ںϲ�ͼƬ�е�λ��
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     std::string  
		* @ note
		*/
		std::string getSite(void);


		/**
		  * @brief  ��ȡ����ͷ�ڲ�����
		  * @date   2018��9��18��
		  * @return     cv::Mat  �ڲ���������
		  */
		cv::Mat	getIntrinsicMatrix();


		/**
		  * @brief  ��������ͷ�ڲ�����
		  * @date   2018��9��18��
		  * @param[in]  cv::Mat intrinsic_matrix  �ڲ���������
		  */
		void setIntrinsicMatrix(cv::Mat &intrinsic_matrix);


		/**
		  * @brief  ��ȡ��ͷ�������
		  * @author admin
		  * @date   2018��9��18��
		  * @param[out]
		  * @return     cv::Mat  �������
		  */
		cv::Mat	getDistortionCoeffs();


		/**
		  * @brief  ���þ�ͷ�����������
		  * @author admin
		  * @date   2018��9��18��
		  * @param[in]  cv::Mat & distortion_coeffs  �������
		  */
		void setDistortionCoeffs(cv::Mat &distortion_coeffs);


		/*!
		* @ brief  ��ȡ��һ֡ͼƬ
		* @ author ybc
		* @ date   2020��5��18��
		* @ return     cv::Mat  
		* @ note
		*/
		Mat getlastimage();

		bool close_;
		private:
			//�������
			int							id_;						/*! <��� */
			int							port_;						/*! <�˿ں� */
			std::string					ip_;						/*! <IP��ַ */
			std::string					user_;					    /*! <�û��� */
			std::string					pwd_;						/*! <���� */
			std::string					site_;						/*! <λ�� */
			cv::Mat						intrinsic_matrix_;			/*! <����ͷ�ڲ����� */
			cv::Mat						distortion_coeffs_;			/*! <��ͷ������� */

			//�������
			LONG						lUserID_;					/*! <��� */
			NET_DVR_DEVICEINFO_V30		struDeviceInfo_;			/*! <�豸��Ϣ */
			LONG						lRealPlayHandle_;			/*! <���ž�� */
			HWND						hWnd_;						/*! <��� */

			//���ò���
			Params						param_;						/*! <���ò����� */
			std::vector<std::string>    test_image_path_;			/*! <��������λ�� */

			//��־λ
			bool                        path_loaded_;
			bool						first_grab_;				/*! <ץ��һ֡ */	

			//���в���
			Timer						grab_time_;					/*! <ץͼʱ�� */
			int							frame_index_;				/*! <֡�� */
			cv::Mat						image_;						/*! <����ͼ�� */

			//�̴߳���
			std::list<Mat>		        msgRecvQueueGrabeMat_;			/*! <��������ͼ����� */
			std::mutex					grab_image_mutex_;				/*! <����ͼƬ�� */

	}; // end class camera



} // end namespace VisionMonitor

#endif	// __CAMERA_H__
