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
 * @brief 监控类，加载监控参数，初始化监控设备
 */
namespace VisionMonitor
{
	/**
* @brief 监测模块所需要的参数
*/

	class Monitor
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
		std::vector<Camera>	    cameras_;				/*!< 相机列表 */
		std::thread				monitorThread_;			/*!< 监测进程 */

	};





}
#endif	// __MONITOR_HH__
