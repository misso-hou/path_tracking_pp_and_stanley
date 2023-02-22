cmake_minimum_required(VERSION 3.10)
project(path_tracking_test)
set(CMAKE_CXX_STANDARD 11)
#指定库路径
file(GLOB_RECURSE PYTHON2.7_LIB "/usr/lib/python2.7/config-x86_64-linux-gnu/*.so")
#指定头文件路径
set(PYTHON2.7_INLCUDE_DIRS "/usr/include/python2.7")
#添加头文件到工程
include_directories(include ${PYTHON2.7_INLCUDE_DIRS})
#这样配置可以使得matplotlib.h中找到python.h文件，通常这样设置就可以。
file(GLOB_RECURSE SRC_FILES src/*.cpp)
add_executable(path_tracking ${SRC_FILES})
#add header file into project
target_include_directories(path_tracking PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#添加库文件到工程
target_link_libraries(path_tracking ${PYTHON2.7_LIB})
