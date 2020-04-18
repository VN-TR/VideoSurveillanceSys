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

// DECLARATION PART
/**
 * @brief ����࣬���ؼ�ز�������ʼ������豸
 */
namespace VisionMonitor
{

	class Monitor
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

		/**
		* @brief �������ģ��
		*/
		void start();

		/**
		  * @brief  �رտ�λ���ģ��
		  * @date   2018��9��18��
		  * @return ���عرս��
		  * @retval true �رճɹ�
		  * @retval false �رղ��ɹ�
		  */
		bool close();


		void monitorThread();


		/**
		  * @brief ��ȡ���ò���
		  * @return ���ò�������ָ��
		  * @retval Params* ����ָ��
		  */
		Params* getParam();

		/**
		  * @brief ��ȡ�������
		  * @return �������ָ��
		  * @retval std::vector<Camera>& �������ָ��
		  */
		std::vector<Camera>& getCameras();

		/**
		  * @brief ��ȡ����ͷ����
		  * @return ��������ͷ����
		  * @retval int ����ͷ����
		  */
		size_t getCamerasNumber();

		/**
		* @brief  ��������ͷģ�飬��ʼ������ͷ�Ϳ�λ���������ͷ�Ϳ�λ��ʼ����������true�����򷵻�false����ע�⣺�������Ҫ����Լ30s~60s��ʱ�䣩
		* @date   2018��9��18��
		*/
		bool initMointor();

		/**
		* @brief  �����ʱ�ļ����ͷſռ䣬�رռ����򣬳ɹ�����true, ���򷵻�false
		* @date   2018��9��18��
		* @return �ɹ�����true, ���򷵻�false
		*/
		bool closeMonitor();


		std::thread& getThread();


	private:
		/**
		  * @brief ��xml�ļ����������в���
		  * @return ���ز�����ȡ���
		  * @retval true ������ȡ�ɹ�
		  * @retval false ������ȡʧ��
		  */
		bool loadParames();

		/**
		* @brief ���ģ�鴦���߳�
		*/
		//void monitorThread();

		/**
		  * @brief  ��ʾ����
		  * @author admin
		  * @date   2018��9��19��
		  * @param[in]  Params & param  ��������
		  */
		void showParames(Params &param);

		/**
		  * @brief  ���ز�����ֵ
		  * @author admin
		  * @date   2018��9��19��
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

		bool					is_start;		/*!< ����ѿ��� */
		Params				    param_;		    /*!< ������� */
		std::vector<Camera>	    cameras_;       /*!< ����б� */
		std::thread				monitorThread_; /*!< ������ */
	};


	/**
* @brief ���ģ������Ҫ�Ĳ���
*/
	struct Params
	{
		bool image_log_switch = true;		        /*!< ��¼ͼƬ��־���� */
		bool display_switch = true;		            /*!< ͼ����ʾ������� */
		bool display_state_switch = true;				/*!< ͼƬ����ʾ��λ״̬����*/
		int data_from = 1;		        /*!< 1��ʵʱ 0����̬����./test_image */
		int	connect_time = 2000;	            /*!< ����ʱ�� */
		int	reconnect_time = 10000;	        /*!< ����ʱ�� */
		int grab_image_type = IMAGE_TYPE_JPG;   /*!< ץ��ͼƬ������ */
		int storage_init_count = 10;				/*!< ��λ��ʼ������ */
		int storage_state_change_count = 30;				/*!< ��λ״̬�л������жϽ����ͬ���� */
		float havegoods_percentage_threshould = 0.6f;				/*!< �жϿ�λ�л�ʱ��������İٷֱȷ�ֵ */
		float nogoods_percentage_threshould = 0.2f;				/*!< �жϿ�λΪ��ʱ��������İٷֱȷ�ֵ */
		int area_threshod_times = 4;				/*!< �������������ֵ,���Ϊͼ�������1/area_threshod */
		float target_percentage_threshod = 0.3f;				/*!< ��Ŀ���ڸǿ�λ�ٷֱȴ���valueʱ���ж�Ϊ�л� */
		float lbp_cascade_scaleFactor = 1.1f;				/*!< �������������߶�ϵ�� */
		int lbp_cascade_minNeighbors = 2;				/*!< ���������������Ҫ�������� */
		int image_resize_times = 4;				/*!< ԭͼ��С���� */
		float image_show_size = 1.5f;             /*!< ��ʾ�����С */
	};


}
#endif	// __MONITOR_HH__
