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
 * @brief 监控类，加载监控参数，初始化监控设备
 */
namespace VisionMonitor
{

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

		/**
		* @brief 启动监测模块
		*/
		void start();

		/**
		  * @brief  关闭库位监控模块
		  * @date   2018年9月18日
		  * @return 返回关闭结果
		  * @retval true 关闭成功
		  * @retval false 关闭不成功
		  */
		bool close();


		void monitorThread();


		/**
		  * @brief 获取配置参数
		  * @return 配置参数对象指针
		  * @retval Params* 对象指针
		  */
		Params* getParam();

		/**
		  * @brief 获取相机对象
		  * @return 相机对象指针
		  * @retval std::vector<Camera>& 相机对象指针
		  */
		std::vector<Camera>& getCameras();

		/**
		  * @brief 获取摄像头数量
		  * @return 返回摄像头数量
		  * @retval int 摄像头数量
		  */
		size_t getCamerasNumber();

		/**
		* @brief  启动摄像头模块，初始化摄像头和库位，如果摄像头和库位初始化正常返回true，否则返回false。（注意：这个过程要花费约30s~60s的时间）
		* @date   2018年9月18日
		*/
		bool initMointor();

		/**
		* @brief  清除临时文件，释放空间，关闭监测程序，成功返回true, 否则返回false
		* @date   2018年9月18日
		* @return 成功返回true, 否则返回false
		*/
		bool closeMonitor();


		std::thread& getThread();


	private:
		/**
		  * @brief 读xml文件，加载运行参数
		  * @return 返回参数读取结果
		  * @retval true 参数读取成功
		  * @retval false 参数读取失败
		  */
		bool loadParames();

		/**
		* @brief 监测模块处理线程
		*/
		//void monitorThread();

		/**
		  * @brief  显示参数
		  * @author admin
		  * @date   2018年9月19日
		  * @param[in]  Params & param  参数对象
		  */
		void showParames(Params &param);

		/**
		  * @brief  加载参数的值
		  * @author admin
		  * @date   2018年9月19日
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

		bool					is_start;		/*!< 监测已开启 */
		Params				    param_;		    /*!< 处理参数 */
		std::vector<Camera>	    cameras_;       /*!< 相机列表 */
		std::thread				monitorThread_; /*!< 监测进程 */
	};


	/**
* @brief 监测模块所需要的参数
*/
	struct Params
	{
		bool image_log_switch = true;		        /*!< 记录图片日志开关 */
		bool display_switch = true;		            /*!< 图像显示输出开关 */
		bool display_state_switch = true;				/*!< 图片上显示库位状态开关*/
		int data_from = 1;		        /*!< 1：实时 0：静态数据./test_image */
		int	connect_time = 2000;	            /*!< 连接时间 */
		int	reconnect_time = 10000;	        /*!< 重连时间 */
		int grab_image_type = IMAGE_TYPE_JPG;   /*!< 抓拍图片的类型 */
		int storage_init_count = 10;				/*!< 库位初始化次数 */
		int storage_state_change_count = 30;				/*!< 库位状态切换连续判断结果相同次数 */
		float havegoods_percentage_threshould = 0.6f;				/*!< 判断库位有货时背景差异的百分比阀值 */
		float nogoods_percentage_threshould = 0.2f;				/*!< 判断库位为空时背景差异的百分比阀值 */
		int area_threshod_times = 4;				/*!< 干扰物体面积阀值,面积为图像面积的1/area_threshod */
		float target_percentage_threshod = 0.3f;				/*!< 大目标掩盖库位百分比大于value时，判定为有货 */
		float lbp_cascade_scaleFactor = 1.1f;				/*!< 级联分类器检测尺度系数 */
		int lbp_cascade_minNeighbors = 2;				/*!< 级联分类器检测需要的邻域数 */
		int image_resize_times = 4;				/*!< 原图缩小倍数 */
		float image_show_size = 1.5f;             /*!< 显示界面大小 */
	};


}
#endif	// __MONITOR_HH__
