cmake_minimum_required(VERSION 3.0)
project(UseVnPkg)

find_package(VnPkgGuide 0.0.1 REQUIRED)
find_package(OpenCV REQUIRED)

message("Version of VnPkgGuide found: ${VnPkgGuide_VERSION}")
message("Found VnPkgGuide include dirs: ${VnPkgGuide_INCLUDE_DIRS}")
message("Found VnPkgGuide libs : ${VnPkgGuide_LIBS}")

add_executable(use_vn_pkg use_vn_pkg.cc)
target_include_directories(use_vn_pkg 
    PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(use_vn_pkg ${VnPkgGuide_LIBS})
