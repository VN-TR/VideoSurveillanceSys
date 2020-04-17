# 设置：生成动态库
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
# 设置：Windows 动态库模式下导出接口，参看 VnPkgGuide/vn_predef.hh 文件
if(MSVC)
  add_definitions(-DEXE_EXPORT)
  add_definitions(-D_WINDOWS)
  add_definitions(-DUSE_CAFFE)
  add_definitions(-DUSE_CUDA)
  add_definitions(-CMAKE_INTDIR="Release")
endif()

# 获取库的源文件列表
file(GLOB_RECURSE ALL_SOURCES 
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c*)


# 获取库的头文件列表
file(GLOB_RECURSE ALL_HEADERS 
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h*)

# 添加库
add_library(${PROJECT_NAME} ${ALL_SOURCES} ${ALL_HEADERS})
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/libs/Tensorflow/tensorflow.lib
  ${PROJECT_SOURCE_DIR}/libs/Tensorflow/tensorflow_framework.lib
  ${PROJECT_SOURCE_DIR}/libs/Openpose/openpose.lib
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe3rdparty/lib/gflags.lib
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe3rdparty/lib/glog.lib
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe/lib/caffe.lib
  ${PROJECT_SOURCE_DIR}/thirdparty/windows/caffe/lib/caffeproto.lib
  ${PROJECT_SOURCE_DIR}/libs/HCNetSDK/GdiPlus.lib
  ${PROJECT_SOURCE_DIR}/libs/HCNetSDK/HCCore.lib
  ${PROJECT_SOURCE_DIR}/libs/HCNetSDK/HCNetSDK.lib
  ${PROJECT_SOURCE_DIR}/libs/HCNetSDK/PlayCtrl.lib
  )

# 给头文件和源文件分组，方便在Visual Studio里查看
if(MSVC)
  foreach(_source IN ITEMS ${ALL_SOURCES})
    get_filename_component(_source_path "${_source}" PATH)
    file(RELATIVE_PATH _source_path_rel 
      "${CMAKE_CURRENT_SOURCE_DIR}" "${_source_path}")
    string(REPLACE "/" "\\" _source_group_path "${_source_path_rel}")
    source_group("Source Files\\${_source_group_path}" FILES "${_source}")
  endforeach()
  foreach(_header IN ITEMS ${ALL_HEADERS})
    get_filename_component(_header_path "${_header}" PATH)
    file(RELATIVE_PATH _header_path_rel 
      "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/" "${_header_path}")
    string(REPLACE "/" "\\" _group_path "${_header_path_rel}")
    source_group("Header Files\\${_group_path}" FILES "${_header}")
  endforeach()
endif(MSVC)

# 安装 ...
## 设定导出目标的属性
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets  # 自行设定库目标的名称
  LIBRARY DESTINATION lib        # 动态库目标地址：         ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION lib        # 静态库目标地址：         ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION bin        # 可执行文件/dll目标地址：  ${CMAKE_INSTALL_PREFIX}/bin
  INCLUDES DESTINATION include   # 头文件目标地址：         ${CMAKE_INSTALL_PREFIX}/include
  )
## 安装包含版本号信息的 cmake 文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
  )
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
  )
## 安装包含库目标信息的 cmake 的文件
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Config.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
  )
## 安装定义一些习惯性变量的文件
file(WRITE ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config-var.cmake
  "
get_filename_component(_PROJ_IMPORT_PREFIX \"\$\{CMAKE_CURRENT_LIST_FILE\}\" PATH)
get_filename_component(_PROJ_IMPORT_PREFIX \"\$\{_PROJ_IMPORT_PREFIX\}\" PATH)
get_filename_component(_PROJ_IMPORT_PREFIX \"\$\{_PROJ_IMPORT_PREFIX\}\" PATH)
get_filename_component(_PROJ_IMPORT_PREFIX \"\$\{_PROJ_IMPORT_PREFIX\}\" PATH)
if(_PROJ_IMPORT_PREFIX STREQUAL \"/\")
  set(_PROJ_IMPORT_PREFIX \"\")
endif()
list(APPEND ${PROJECT_NAME}_INCLUDE_DIR
  \$\{_PROJ_IMPORT_PREFIX\}/include
)
set(${PROJECT_NAME}_INCLUDE_DIRS \$\{${PROJECT_NAME}_INCLUDE_DIR\})

list(APPEND ${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME})
set(${PROJECT_NAME}_LIBS \$\{${PROJECT_NAME}_LIBRARIES\})
set(${PROJECT_NAME}_LIBRARY \$\{${PROJECT_NAME}_LIBRARIES\})
")
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config-var.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
  )
