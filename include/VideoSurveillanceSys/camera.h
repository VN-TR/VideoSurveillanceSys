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
#include "VideoSurveillanceSys/monitor.h"
#include "HCNetSDK/HCNetSDK.h"
#include "HCNetSDK/plaympeg4.h"
#include <thread>

// ʱ������궨��;
#define GET_YEAR(_time_)      (((_time_)>>26) + 2000) 
#define GET_MONTH(_time_)     (((_time_)>>22) & 15)
#define GET_DAY(_time_)       (((_time_)>>17) & 31)
#define GET_HOUR(_time_)      (((_time_)>>12) & 31) 
#define GET_MINUTE(_time_)    (((_time_)>>6)  & 63)
#define GET_SECOND(_time_)    (((_time_)>>0)  & 63)
#define XML_BUF 3*1024*1024
#define IDC_STATIC_PLAY = 1000;


// DECLARATION PART
namespace VisionMonitor
{
	/**
	 * @brief �����
	 */
	class Camera
	{
	public:

		Camera();

		~Camera();

		void monitorThread();


		/**
		  * @brief ��ʼ���������λ,��������ʱ��������ʱ�䣬ע���豸����ȡ�豸������Ϣ������ѵ�����
		  * @return ���������ʼ�����
		  * @retval true �ɹ�
		  * @retval false ���ɹ�
		  */
		bool initialize(const Params &param);


		void close();
		/**
		* @brief ��������ͷ���
		* @return ��
		* @retval void
		*/
		std::thread* startMonitor();


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
		  * @brief ��ȡ�豸��˿ں�
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

		/**
		  * @brief ץ��
		  * @param[in] int ipicType ץͼ����
		  * @param[out] std::string ipicType ͼƬ����
		  * @return ����ץ��ͼƬ
		  * @retval cv::Mat ץ��ͼƬ
		  */
		cv::Mat grabbingFrame(Params &params, int ipicType, std::string &img_name);

		/**
		  * @brief  ��ȡ����ͷ��ǰ��ͼ��
		  * @author admin
		  * @date   2018��9��3��
		  * @return cv::Mat  ����ͷ��ǰ��ͼ��
		  */
		cv::Mat getCameraImage();



		/**
		  * @brief  ��ӡ��������м�ͼ��
		  * @date   2018��9��20��
		  * @param[in]  int r  ͼ����
		  * @param[in]  int c  ͼ����
		  */
		void display(int r, int c, const Params &param);

		/**
		  * @brief  ��LOGO���뵽ͼƬ��
		  * @date   2018��9��20��
		  * @param[in]  int rowStart  ͼ����
		  * @param[in]  int colStart  ͼ����
		  */
		void InsertLogo(Mat image, Mat logoImage, int rowStart, int colStart);


	private:


		/**
		  * @brief ���豸ע��
		  * @return ����ע����
		  * @retval ture �ɹ�
		  * @retval false ���ɹ�
		  */
		bool doLogin();

		/**
		  * @brief ��ȡ�豸��ͨ����Դ
		  * @return ��
		  * @retval void
		  */
		void doGetDeviceResoureCfg();

		/**
		  * @brief ע����¼
		  * @return ��
		  * @retval void
		  */
		void doLoginOut();



	private:

		// ���ò���;
		int							id_;						/*! <��� */
		int							port_;						/*! <�˿ں� */
		std::string					ip_;						/*! <IP��ַ */
		std::string					user_;					    /*! <�û��� */
		std::string					pwd_;						/*! <���� */
		

		cv::Mat                     image_;                     /*! <���ͼ�� */
		cv::Mat                     Title_image;                /*! <����ͼƬ */
		cv::Mat                     Inform_car_image;           /*! <��ʾ����ͼƬ */
		cv::Mat                     Inform_human_image;         /*! <��ʾ�˵�ͼƬ */
		cv::Mat                     Inform_good_image;          /*! <��ʾ����ͼƬ */
		cv::Mat						shadow_;
		cv::Mat						background_;				/*! <����ͼƬ */
		cv::Mat						camera_cell_merge_;			/*! <����������λ�����ϲ�ͼ��*/
		cv::Mat						blob_target_;				/*! <��Ŀ���ͼ�� */
		cv::Mat						contours_img_;				/*! <��Ŀ��������ͼ�� */
		cv::Mat						convex_img_;				/*! <Ŀ��͹�����ͼ�� */
		cv::Mat						intrinsic_matrix_;			/*! <����ͷ�ڲ����� */
		cv::Mat						distortion_coeffs_;			/*! <��ͷ������� */
		float						background_percentage_;		/*! <�����ٷֱ� */
		float						shadow_percentage_;			/*! <��Ӱ�ٷֱ� */
		MonitorState                monitor_action_;            /*! <��⹦�� */
		std::vector<Cell>		    vct_cell_;					/*! <��λ��Ϣ�б� */

		DeeplearningDetector        deeplearning_detector_;     /*! <���ѧϰ */
		std::vector<Saveditem>      AI_result;                  /*! <���ѧϰ�Ľ�� */
		
		
		// ���в���;
		bool						isLogin_;					/*! <�Ƿ�Ϊ��¼״̬ */
		LONG						lPlayHandle_;				/*!< */
		LONG						user_id_;					/*!< */
		LONG						handle_;					/*!< */
		LOCAL_DEVICE_INFO			struDeviceInfo_;			/*!< */
		NET_VCA_FIELDDETECION		struFieldDetection_;		/*!< */
		NET_DVR_CHANNEL_GROUP		struFielDetectionCond_;	    /*!< */
		NET_DVR_SETUPALARM_PARAM	struAlarmParam_;			/*!< */

		Params                      param_;
		bool                        path_loaded_;
		std::vector<std::string>    test_image_path_;
		int                         frame_index_;

		//�ݴ����;
		bool                        unsafe_Forklift;            /*! <�泵�����λ */
		bool                        unsafe_Good;                /*! <������� */
		bool                        unsafe_Human;               /*! <�˽����λ */



	}; // end class camera

} // end namespace VisionMonitor

#endif	// __CAMERA_H__
