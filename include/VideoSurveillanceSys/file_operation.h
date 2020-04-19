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
 * @brief �ļ������࣬���е��ļ���ز�������������ȡ�ļ��б�Ȳ���
 */
class FileOperation
{

public:
	FileOperation();

	~FileOperation();

	/**
	  * @brief ��ȡfilePath·���µ������ļ������Ʊ��浽vctFileName
	  * @param[in] const std::string filePath �ļ�·��
	  * @param[out] std::vector<std::string> & vctFileName �ļ��������б�
	  * @return �ļ����б�
	  * @retval bool true ·������
	  * @retval bool false ·��������
	  */
	bool getFolderName(const std::string filePath, std::vector<std::string> &vctFileName);

	/**
	  * @brief ��filePath·��������fileSuf��׺�����ļ����浽vctFileName
	  * @param[in] const std::string filePath �ļ�·��
	  * @param[in] const std::string fileSuf �ļ���׺��
	  * @param[out] std::vector<std::string> & vctFileName fileSuf��׺�����ļ������б�
	  * @return ��
	  * @retval void
	  */
	void getFileNameList(const std::string filePath, const std::string fileSuf, std::vector<std::string> &vctFileName);

	/**
	  * @brief ����filePath���ļ��к�׺��ΪfileSuf���ļ��У�ʱ���near_time���µ�file_count���ļ�
	  * @param[in] const std::string filePath �ļ�·��
	  * @param[in] const std::string fileSuf �ļ���׺��
	  * @param[in] const int64_t near_time ʱ��
	  * @param[in] const int file_count �ļ�������
	  * @param[out] std::vector<std::string> & vctFileName ʱ���near_time���µ�file_count���ļ�,���filePath��׺��ΪfileSuf���ļ�����file_count,��vctFileName.size()ȡ��С����Ŀ
	  * @return ��
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
	  * @brief  �ж��Ƿ���".."Ŀ¼��"."Ŀ¼
	  * @author admin
	  * @date   2018��9��19��
	  * @param[in]  const char * path �ļ�·��
	  * @return     bool  ���ؽ������".."Ŀ¼��"."Ŀ¼����true,���򷵻�false
	  */
	bool is_special_dir(const char *path);

	/**
	  * @brief  �ж��ļ�������Ŀ¼�����ļ�
	  * @author admin
	  * @date   2018��9��19��
	  * @param[in]  int attrib
	  * @return     bool  ��Ŀ¼�����ļ�����true,���򷵻�false
	  */
	bool is_dir(int attrib);

	/**
	  * @brief  ��ʾɾ��ʧ��ԭ��
	  * @author admin
	  * @date   2018��9��19��
	  * @param[in]  const char * file_name  �ļ�����·��
	  */
	void show_error(const char *file_name = NULL);

	/**
	  * @brief  �ݹ�����Ŀ¼�е��ļ���ɾ��
	  * @author admin
	  * @date   2018��9��19��
	  * @param[in]  std::string path  �ļ���·��
	  */
	void removeDirectory(std::string path);

	/**
	  * @brief  ����file_name������·��
	  * @author admin
	  * @date   2018��9��19��
	  * @param[out]
	  * @param[in]  const char * path
	  * @param[in]  const char * file_name
	  * @param[in]  char * file_path
	  * @return     void
	  */
	void get_file_path(const char *path, const char *file_name, char *file_path);

	/**
	  * @brief  ��ȡĿ¼�����б����ļ��������ļ����Ʒ��࣬����ÿ�����������ѡ��һ���ļ�
	  * @author admin
	  * @date   2018��9��19��
	  * @param[in]  const std::string filePath  �ļ���·��
	  * @param[in]  const std::string fileSuf  ��Ҫѡ����ļ���׺��
	  * @param[in]  std::vector<std::string> & vctfile  ���ѡ����ļ��б�
	  */
	void getFileMap(
		const std::string filePath,
		const std::string fileSuf,
		std::vector<std::string> &vctfile
	);

	/**
	  * @brief �ҳ�filePath·����������fileSufΪ��׺�����ļ��е����µ��ļ����ļ����ƴ���filename
	  * @param[in] const std::string filePath �ļ�·��
	  * @param[in] const std::string fileSuf �ļ���׺��
	  * @param[out] std::string & filename fileSuf��׺�����ļ������µ��ļ���
	  * @return ��
	  * @retval bool true ·������
	  * @retval bool false ·��������
	  */
	bool getLastFileName(const std::string filePath, const std::string fileSuf, std::string &filename);

	/**
	  * @brief ���filePath���ļ����Ƿ����,����������򴴽�
	  * @param[in] const std::string filePath �ļ�·��
	  * @return ��
	  * @retval bool true �����ɹ�
	  * @retval bool false �������ɹ�
	  */
	bool checkAndCreateDir(const std::string filePath);

}; // end class FileOperation
