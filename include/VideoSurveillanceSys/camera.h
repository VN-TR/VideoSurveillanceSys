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
#include "VideoSurveillanceSys/object_detection.h"
#include <openpose/headers.hpp>
#include <thread>
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


		std::thread* Camera::startMonitor();


		void Camera::monitorThread();

		cv::Mat Camera::grabbingFrame(Params &params, std::string &img_name);

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

		Mat getlastimage();
		



		private:
			//�������
			int							id_;						/*! <��� */
			int							port_;						/*! <�˿ں� */
			std::string					ip_;						/*! <IP��ַ */
			std::string					user_;					    /*! <�û��� */
			std::string					pwd_;						/*! <���� */
			cv::Mat						intrinsic_matrix_;			/*! <����ͷ�ڲ����� */
			cv::Mat						distortion_coeffs_;			/*! <��ͷ������� */

			//���в���
			std::vector<Saveditem>      AI_result_;



			bool                        path_loaded_;
			std::vector<std::string>    test_image_path_;

			int							frame_index_;
			Params						param_;
			ObjectDetection				object_detection_;
			
			LONG						lUserID_;
			NET_DVR_DEVICEINFO_V30		struDeviceInfo_;
			LONG						lRealPlayHandle_;
			
			HWND						hWnd_;
			cv::Mat						image_;
			Mat display_image;
			cv::Mat						skeleton_image_;
			cv::Mat						Title_image_;
			cv::Mat						Inform_car_image_;
			cv::Mat						Inform_human_image_;
			cv::Mat						Inform_good_image_;

	}; // end class camera

} // end namespace VisionMonitor

#endif	// __CAMERA_H__
