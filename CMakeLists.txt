cmake_minimum_required(VERSION 3.1)

# 指定库名和版本号
project(VideoSurveillanceSys
  VERSION 0.0.1
  )

# 使用现代 C++（C++ 11及以上）
set(CMAKE_CXX_STANDARD 14)

# 第三方依赖库
find_package(OpenCV REQUIRED)

# 头文件目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${VnUtils_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/opencv/include
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe/include
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe/include2
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe3rdparty/include
  ${PROJECT_SOURCE_DIR}/thirdparty/tensorflow/cpu1.14/include
  ${PROJECT_SOURCE_DIR}/thirdparty/tensorflow/cpu1.14/include/tensorflow
  )

# 设置：生成库文件的地址
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 设置：生成可执行文件和 dll 的地址
# 所有子项目的 dll 放在一处，方便调试时加载 dll 文件
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 设置：Debug 模式下库文件名添加字符 'd'，以区分 Release 库
set(CMAKE_DEBUG_POSTFIX "d")

# 子目录：库源文件
add_subdirectory(src)

# 子目录：测试
add_subdirectory(tests)

## 以下子目录为可选项

# 子目录：实例代码（演示如何使用库，不必安装二进制）
#       如果代码足够简单，或注释足够清楚，可不用提供示例代码
add_subdirectory(examples)

# 子目录：应用代码（供用户使用，应安装）
#       如果不需提供应用供用户使用，不必添加此项
add_subdirectory(apps)


# 安装头文件（拷贝整个文件夹即可）
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include                # 目标地址：${CMAKE_INSTALL_PREFIX}/include
  )

# 安装数据文件和文档文件
#install(DIRECTORY data DESTINATION ${CMAKE_INSTALL_PREFIX})
#install(DIRECTORY doc DESTINATION ${CMAKE_INSTALL_PREFIX})