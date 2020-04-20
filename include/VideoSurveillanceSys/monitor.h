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

		
		void start();

		void monitorThread();

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
		std::vector<Camera>	    cameras_;				/*!< ����б� */
		std::thread				monitorThread_;			/*!< ������ */

	};





}
#endif	// __MONITOR_HH__
