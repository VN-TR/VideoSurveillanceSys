/**Copyright (C) 2018-2018 Visionnav Robotics Limited. All right reserved
* @file: operationFile.cc
* @version: V 1.0.0
* @author: bcyang@Visionnav.com;
* @date: 2020-04-19
* @brief: 文件操作类函数实现;
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

	//先将要切割的字符串从string类型转换为char*类型;
	char * strs = new char[str.length() + 1];
	strcpy(strs, str.c_str());

	char * d = new char[delim.length() + 1];
	strcpy(d, delim.c_str());

	char *p = strtok(strs, d);
	while (p) {
		std::string s = p; //分割得到的字符串转换为string类型;
		res.push_back(s); //存入结果数组;
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

	// 打开FTP文件夹，获取文件夹列表;
	if ((hFileHandle = _findfirst(fpath.c_str(), &fileInfo)) == -1L)
	{
		std::cout << "打开FTP文件目录失败" << std::endl;
		return false;
	}
	else
	{
		do
		{
			// 判断当前文件是否为子目录;
			if (fileInfo.attrib & _A_SUBDIR)
			{
				// 判断当前子目录是否为根目录;
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

	// 读取monitor/cameraXX/backgroub下所有的背景文件并根据名称分类
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

	// 从每个分类中随机选一张图片
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

	// 文件按时间排序;
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

	// 找离neartime时间最近的文件的下标;
	near_time_index = -1;
	for (int i = 0; i < size; i++)
	{
		min_time_diff = fileCreateTime[i] - near_time;
		if (min_time_diff < max_time)
		{
			near_time_index = i;
		}
	}

	// 找离neartime时间最近的file_count个文件;
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
	// 断言文件路径和文件后缀名不为空，图像名称为空;
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

	// 删除文件夹下其它图片;
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

	// 检查cvlog文件夹是否存在,如果不存在则创建;
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

			if (is_dir(file_info.attrib))//如果是目录，生成完整的路径
			{
				get_file_path(path.c_str(), file_info.name, tmp_path);

				removeDirectory(tmp_path); //递归删除目录中的内容

				tmp_path[strlen(tmp_path) - 2] = '\0';
				if (file_info.attrib == 20)
				{
					printf("This is system file, can't delete!\n");
				}
				else
				{
					//删除空目录，必须在递归返回前调用_findclose,否则无法删除目录
					if (_rmdir(tmp_path) == -1)
					{
						show_error();//目录非空则会显示出错原因
					}
				}
			}
			else
			{
				strcpy_s(tmp_path, sizeof(char) * _MAX_PATH, path.c_str());

				tmp_path[strlen(tmp_path) - 1] = '\0';

				strcat_s(tmp_path, sizeof(char) * _MAX_PATH, file_info.name);//生成完整的文件路径

				if (remove(tmp_path) == -1)
				{
					show_error(file_info.name);
				}
			}
		}
		_findclose(f_handle);//关闭打开的文件句柄，并释放关联资源，否则无法删除空目录
	}
	else
	{
		show_error();//若路径不存在，显示错误信息
	}
}
