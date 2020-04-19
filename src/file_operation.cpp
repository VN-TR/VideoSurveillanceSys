/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
* @file: operationFile.cc
* @version: V 1.0.0
* @author: bcyang@Visionnav.com;
* @date: 2020-04-19
* @brief: �ļ������ຯ��ʵ��;
* @details:
* @verbatim:
*/

#include "VideoSurveillanceSys/file_operation.h"

#include <windows.h>
#include <iostream>
#include <assert.h>
#include <io.h>
#include <direct.h>
#include <map>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>

std::vector<std::string> split(const std::string& str, const std::string& delim)
{
	std::vector<std::string> res;
	if ("" == str) return res;

	//�Ƚ�Ҫ�и���ַ�����string����ת��Ϊchar*����;
	char * strs = new char[str.length() + 1];
	strcpy(strs, str.c_str());

	char * d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());

	char *p = strtok(strs, d);
	while (p) {
		std::string s = p; //�ָ�õ����ַ���ת��Ϊstring����;
		res.push_back(s); //����������;
		p = strtok(NULL, d);
	}

	delete[] strs;
	delete[] d;
	delete p;

	return res;
}

FileOperation::FileOperation()
{
	// to do nothing
}

FileOperation::~FileOperation()
{
	// to do nothing
}

bool FileOperation::getFolderName(const std::string filePath, std::vector<std::string> &vctFileName)
{
	assert("" != filePath && 0 == vctFileName.size());

	long hFileHandle;
	_finddata_t fileInfo;
	std::string fpath;
	fpath = filePath;
	fpath += "\\*";

	// ��FTP�ļ��У���ȡ�ļ����б�;
	if ((hFileHandle = _findfirst(fpath.c_str(), &fileInfo)) == -1L)
	{
		std::cout << "��FTP�ļ�Ŀ¼ʧ��" << std::endl;
		return false;
	}
	else
	{
		do
		{
			// �жϵ�ǰ�ļ��Ƿ�Ϊ��Ŀ¼;
			if (fileInfo.attrib & _A_SUBDIR)
			{
				// �жϵ�ǰ��Ŀ¼�Ƿ�Ϊ��Ŀ¼;
				//if (strcmp(fileInfo.name, ".") != 0 && strcmp(fileInfo.name, "..") != 0 )
				if (!is_special_dir(fileInfo.name))
				{
					vctFileName.push_back(fileInfo.name);
				}
				else
				{
					// to do nothing
				}
			}
			else
			{
				// to do nothing
			}

		} while (_findnext(hFileHandle, &fileInfo) == 0);
		_findclose(hFileHandle);
		return true;
	}
}

void FileOperation::getFileMap(const std::string filePath, const std::string fileSuf, std::vector<std::string> &vctfile)
{
	std::map<int, std::vector<std::string>> filemap;
	long hImageHandle;
	_finddata_t fileInfo;
	std::string  templetPath = filePath;
	templetPath += "\\*";
	templetPath += fileSuf;
	//std::vector<__time64_t> fileCreateTime;
	std::vector<std::string> strs;

	// ��ȡmonitor/cameraXX/backgroub�����еı����ļ����������Ʒ���
	if ((hImageHandle = _findfirst(templetPath.c_str(), &fileInfo)) != -1L)
	{
		do
		{
			strs.clear();
			strs = split(fileInfo.name, "-");
			int type = atoi(strs[0].c_str());
			std::vector<std::string> names;
			std::string imgPath = filePath + "/" + fileInfo.name;
			if (filemap.end() != filemap.find(type))
			{
				filemap.find(type)->second.push_back(imgPath);
			}
			else
			{
				names.push_back(imgPath);
				filemap.insert(std::make_pair(type, names));
			}
			//fileCreateTime.push_back(fileInfo.time_create);
		} while (0 == _findnext(hImageHandle, &fileInfo));
	}
	_findclose(hImageHandle);

	// ��ÿ�����������ѡһ��ͼƬ
	vctfile.clear();
	vctfile.reserve(filemap.size());
	for each (auto files in filemap)
	{
		int select_index = rand() % files.second.size();
		vctfile.push_back(files.second[select_index]);
	}
}

void FileOperation::getFileNameList(const std::string filePath, const std::string fileSuf, std::vector<std::string> &vctFileName)
{
	LONGLONG hImageHandle;
	_finddatai64_t fileInfo;
	std::string  templetPath = filePath;
	templetPath += "\\*";
	templetPath += fileSuf;
	std::vector<__time64_t> fileCreateTime;
	int size;
	vctFileName.clear();

	if ((hImageHandle = _findfirsti64(templetPath.c_str(), &fileInfo)) != -1L)
	{
		do
		{
			if (is_special_dir(fileInfo.name))continue;

			vctFileName.push_back(fileInfo.name);
			fileCreateTime.push_back(fileInfo.time_create);

		} while (0 == _findnext64(hImageHandle, &fileInfo));
	}

	size = vctFileName.size();

	if (0 == size) return;

	for (int i = 0; i < size - 1; i++)
	{
		for (int j = i + 1; j < size; j++)
		{
			if (fileCreateTime[i] > fileCreateTime[j])
			{
				__time64_t t;
				t = fileCreateTime[i];
				fileCreateTime[i] = fileCreateTime[j];
				fileCreateTime[j] = t;

				std::string n;
				n = vctFileName[i];
				vctFileName[i] = vctFileName[j];
				vctFileName[j] = n;
			}
		}
	}
	_findclose(hImageHandle);
}

void FileOperation::getFileNameListByNearTime(
	const std::string filePath,
	const std::string fileSuf,
	const int64_t near_time,
	const int file_count,
	std::vector<std::string> &vctFileName)
{
	long hImageHandle;
	_finddata_t fileInfo;
	std::string  templetPath = filePath;
	templetPath += "\\*";
	templetPath += fileSuf;
	std::vector<__time64_t> fileCreateTime;
	std::vector<std::string> vctFileNameTemp;
	int size;
	int64_t min_time_diff;
	int max_time = MAXINT;
	int near_time_index;

	vctFileName.clear();
	vctFileNameTemp.clear();

	if ((hImageHandle = _findfirst(templetPath.c_str(), &fileInfo)) != -1L)
	{
		do
		{
			vctFileNameTemp.push_back(fileInfo.name);
			fileCreateTime.push_back(fileInfo.time_create);
		} while (0 == _findnext(hImageHandle, &fileInfo));
	}
	_findclose(hImageHandle);

	size = vctFileNameTemp.size();

	if (size <= file_count)
	{
		return;
	}

	// �ļ���ʱ������;
	for (int i = 0; i < size - 1; i++)
	{
		for (int j = i + 1; j < size; j++)
		{
			if (fileCreateTime[i] < fileCreateTime[j])
			{
				__time64_t t;
				t = fileCreateTime[i];
				fileCreateTime[i] = fileCreateTime[j];
				fileCreateTime[j] = t;

				std::string n;
				n = vctFileNameTemp[i];
				vctFileNameTemp[i] = vctFileNameTemp[j];
				vctFileNameTemp[j] = n;
			}
		}
	}

	// ����neartimeʱ��������ļ����±�;
	near_time_index = -1;
	for (int i = 0; i < size; i++)
	{
		min_time_diff = fileCreateTime[i] - near_time;
		if (min_time_diff < max_time)
		{
			near_time_index = i;
		}
	}

	// ����neartimeʱ�������file_count���ļ�;
	vctFileName.resize(file_count);
	if (near_time_index - file_count >= 0)
	{
		for (int i = near_time_index - file_count; i < file_count; i++)
		{
			vctFileName.push_back(vctFileNameTemp[i]);
		}
	}
	else
	{
		for (int i = size - (file_count - near_time_index); i < size; i++)
		{
			vctFileName.push_back(vctFileNameTemp[i]);
		}
		for (int i = 0; i < near_time_index; i++)
		{
			vctFileName.push_back(vctFileNameTemp[i]);
		}
	}
}

bool FileOperation::getLastFileName(const std::string filePath, const std::string fileSuf, std::string &imageName)
{
	// �����ļ�·�����ļ���׺����Ϊ�գ�ͼ������Ϊ��;
	assert(0 != strcmp("", filePath.c_str()) && 0 != strcmp("", fileSuf.c_str()) && 0 == strcmp("", imageName.c_str()));

	long hImageHandle;
	_finddata_t fileInfo;
	std::string  imagePath = filePath;
	imagePath += "\\*";
	imagePath += fileSuf;
	__time64_t last = 0;

	if ((hImageHandle = _findfirst(imagePath.c_str(), &fileInfo)) != -1L)
	{
		do
		{
			if (last < fileInfo.time_create)
			{
				imageName = fileInfo.name;
				last = fileInfo.time_create;
			}
			else
			{
				// to do nothing
			}
		} while (0 == _findnext(hImageHandle, &fileInfo));
	}
	else
	{
		return false;
	}

	// ɾ���ļ���������ͼƬ;
	if ((hImageHandle = _findfirst(imagePath.c_str(), &fileInfo)) != -1L)
	{
		do
		{
			if (last != fileInfo.time_create)
			{
				std::string  str_fp = filePath + "\\" + fileInfo.name;
				remove(str_fp.c_str());
			}
			else
			{
				// to do nothing
			}
		} while (0 == _findnext(hImageHandle, &fileInfo));
	}
	else
	{
		return false;
	}

	_findclose(hImageHandle);
	return true;
}

bool FileOperation::checkAndCreateDir(const std::string filePath)
{
	struct _stat fileStat;

	// ���cvlog�ļ����Ƿ����,����������򴴽�;
	if ((_stat(filePath.c_str(), &fileStat) == 0) && (fileStat.st_mode & _S_IFDIR))
	{
		//std::cout << filePath << " file exist" << std::endl;
	}
	else
	{
		if (_mkdir(filePath.c_str()) == 0)
		{
			std::cout << "Directory " << filePath << " was successfully created." << std::endl;
		}
		else
		{
			std::cout << "Directory " << filePath << " create failure !" << std::endl;
			std::string error = "Directory " + filePath + " create failure !";
			return false;
		}
	}
	return true;
}

bool FileOperation::is_special_dir(const char *path)
{
	return strcmp(path, "..") == 0 || strcmp(path, ".") == 0;
}

bool FileOperation::is_dir(int attrib)
{
	return attrib == 16 || attrib == 18 || attrib == 20;
}

void FileOperation::show_error(const char * file_name)
{
	errno_t err;

	_get_errno(&err);

	switch (err)
	{
	case ENOTEMPTY:
		printf("Given path is not a directory, the directory is not empty, or the directory is either the current working directory or the root directory.\n");
		break;
	case ENOENT:
		printf("Path is invalid.\n");
		break;
	case EACCES:
		printf("%s had been opend by some application, can't delete.\n", file_name);
		break;
	}
}

void FileOperation::get_file_path(const char *path, const char *file_name, char *file_path)
{

	strcpy_s(file_path, sizeof(char) * _MAX_PATH, path);

	file_path[strlen(file_path) - 2] = '\0';

	strcat_s(file_path, sizeof(char) * _MAX_PATH, "/");

	strcat_s(file_path, sizeof(char) * _MAX_PATH, file_name);

	strcat_s(file_path, sizeof(char) * _MAX_PATH, "\\*");
}

void FileOperation::removeDirectory(std::string  path)
{
	_finddata_t dir_info;

	_finddata_t file_info;

	intptr_t f_handle;

	char tmp_path[_MAX_PATH];

	//path = "monitor/camera0/storage/cell1\\*";

	if ((f_handle = _findfirst(path.c_str(), &dir_info)) != -1L)
	{
		while (_findnext(f_handle, &file_info) == 0)
		{
			if (is_special_dir(file_info.name))
				continue;

			if (is_dir(file_info.attrib))//�����Ŀ¼������������·��
			{
				get_file_path(path.c_str(), file_info.name, tmp_path);

				removeDirectory(tmp_path); //�ݹ�ɾ��Ŀ¼�е�����

				tmp_path[strlen(tmp_path) - 2] = '\0';
				if (file_info.attrib == 20)
				{
					printf("This is system file, can't delete!\n");
				}
				else
				{
					//ɾ����Ŀ¼�������ڵݹ鷵��ǰ����_findclose,�����޷�ɾ��Ŀ¼
					if (_rmdir(tmp_path) == -1)
					{
						show_error();//Ŀ¼�ǿ������ʾ����ԭ��
					}
				}
			}
			else
			{
				strcpy_s(tmp_path, sizeof(char) * _MAX_PATH, path.c_str());

				tmp_path[strlen(tmp_path) - 1] = '\0';

				strcat_s(tmp_path, sizeof(char) * _MAX_PATH, file_info.name);//�����������ļ�·��

				if (remove(tmp_path) == -1)
				{
					show_error(file_info.name);
				}
			}
		}
		_findclose(f_handle);//�رմ򿪵��ļ���������ͷŹ�����Դ�������޷�ɾ����Ŀ¼
	}
	else
	{
		show_error();//��·�������ڣ���ʾ������Ϣ
	}
}
