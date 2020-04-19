/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
 * @file: operation
 * @version:
 * @author: bcyang@Visionnav.com
 * @date: 2020-04-19
 * @brief:
 * @details:
 * @verbatim:
 */

 // INCLUDE PART
#include <vector>
#include <string>

// DECLARATION PART
/**
 * @brief 文件操作类，所有的文件相关操作，创建，获取文件列表等操作
 */
class FileOperation
{

public:
	FileOperation();

	~FileOperation();

	/**
	  * @brief 获取filePath路径下的所有文件夹名称保存到vctFileName
	  * @param[in] const std::string filePath 文件路径
	  * @param[out] std::vector<std::string> & vctFileName 文件夹名称列表
	  * @return 文件夹列表
	  * @retval bool true 路径存在
	  * @retval bool false 路径不存在
	  */
	bool getFolderName(const std::string filePath, std::vector<std::string> &vctFileName);

	/**
	  * @brief 将filePath路径下所有fileSuf后缀名的文件保存到vctFileName
	  * @param[in] const std::string filePath 文件路径
	  * @param[in] const std::string fileSuf 文件后缀名
	  * @param[out] std::vector<std::string> & vctFileName fileSuf后缀名的文件名称列表
	  * @return 无
	  * @retval void
	  */
	void getFileNameList(const std::string filePath, const std::string fileSuf, std::vector<std::string> &vctFileName);

	/**
	  * @brief 查找filePath下文件夹后缀名为fileSuf的文件中，时间距near_time最新的file_count个文件
	  * @param[in] const std::string filePath 文件路径
	  * @param[in] const std::string fileSuf 文件后缀名
	  * @param[in] const int64_t near_time 时间
	  * @param[in] const int file_count 文件的数量
	  * @param[out] std::vector<std::string> & vctFileName 时间距near_time最新的file_count个文件,如果filePath后缀名为fileSuf的文件不足file_count,则vctFileName.size()取最小的数目
	  * @return 无
	  * @retval void
	  */
	void getFileNameListByNearTime(
		const std::string filePath,
		const std::string fileSuf,
		const int64_t near_time,
		const int file_count,
		std::vector<std::string> &vctFileName
	);

	/**
	  * @brief  判断是否是".."目录和"."目录
	  * @author admin
	  * @date   2018年9月19日
	  * @param[in]  const char * path 文件路径
	  * @return     bool  返回结果，是".."目录和"."目录返回true,否则返回false
	  */
	bool is_special_dir(const char *path);

	/**
	  * @brief  判断文件属性是目录还是文件
	  * @author admin
	  * @date   2018年9月19日
	  * @param[in]  int attrib
	  * @return     bool  是目录还是文件返回true,否则返回false
	  */
	bool is_dir(int attrib);

	/**
	  * @brief  显示删除失败原因
	  * @author admin
	  * @date   2018年9月19日
	  * @param[in]  const char * file_name  文件完整路径
	  */
	void show_error(const char *file_name = NULL);

	/**
	  * @brief  递归搜索目录中的文件并删除
	  * @author admin
	  * @date   2018年9月19日
	  * @param[in]  std::string path  文件夹路径
	  */
	void removeDirectory(std::string path);

	/**
	  * @brief  生成file_name的完整路径
	  * @author admin
	  * @date   2018年9月19日
	  * @param[out]
	  * @param[in]  const char * path
	  * @param[in]  const char * file_name
	  * @param[in]  char * file_path
	  * @return     void
	  */
	void get_file_path(const char *path, const char *file_name, char *file_path);

	/**
	  * @brief  读取目录下所有背景文件并根据文件名称分类，并从每个分类中随机选择一个文件
	  * @author admin
	  * @date   2018年9月19日
	  * @param[in]  const std::string filePath  文件夹路径
	  * @param[in]  const std::string fileSuf  需要选择的文件后缀名
	  * @param[in]  std::vector<std::string> & vctfile  随机选择的文件列表
	  */
	void getFileMap(
		const std::string filePath,
		const std::string fileSuf,
		std::vector<std::string> &vctfile
	);

	/**
	  * @brief 找出filePath路径下所有以fileSuf为后缀名的文件中的最新的文件的文件名称存入filename
	  * @param[in] const std::string filePath 文件路径
	  * @param[in] const std::string fileSuf 文件后缀名
	  * @param[out] std::string & filename fileSuf后缀名的文件中最新的文件名
	  * @return 无
	  * @retval bool true 路径存在
	  * @retval bool false 路径不存在
	  */
	bool getLastFileName(const std::string filePath, const std::string fileSuf, std::string &filename);

	/**
	  * @brief 检查filePath下文件夹是否存在,如果不存在则创建
	  * @param[in] const std::string filePath 文件路径
	  * @return 无
	  * @retval bool true 创建成功
	  * @retval bool false 创建不成功
	  */
	bool checkAndCreateDir(const std::string filePath);

}; // end class FileOperation
