cmake_minimum_required(VERSION 3.20.0)
project(planning
    VERSION 0.0.1
    DESCRIPTION "a demo of cmake planning"
    HOMEPAGE_URL "https://github.com/Mxw0323/Vsc_work.git"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
enable_testing()


# 变量设置
# set(CMMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin) # Windows上使用
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
# set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

set(Process_Dir ${CMAKE_SOURCE_DIR}/src/process)
set(Pnc_Map_Dir ${CMAKE_SOURCE_DIR}/src/pnc_map)
set(Show_Result_Dir ${CMAKE_SOURCE_DIR}/src/show_result)

set(EasyX_Inc_Dir ${CMAKE_SOURCE_DIR}/third_party/easyx_64/include)
set(EasyX_Lib_Dir ${CMAKE_SOURCE_DIR}/third_party/easyx_64/lib)

set(EIGEN3_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/eigen-3.4.0)
list(APPEND CMAKE_MODULE_PATH ${EIGEN3_INCLUDE_DIR}/cmake)
message("CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
find_package(Eigen3 3.4 REQUIRED)
if(TARGET Eigen3::Eigen)
    message("***********************Eigen3 found***********************")
endif()


set(Python_Dir D:/anaconda3/include)
set(Numpy_Dir D:/anaconda3/Lib/site-packages/numpy/core/include)
set(Python_Lib D:/anaconda3/libs/python39.lib)
# set(Numpy_Lib D:/anaconda3/Lib/site-packages/numpy/core/lib/npymath.lib)

# 添加子目录
add_subdirectory(${CMAKE_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_SOURCE_DIR}/test/map_test)
add_subdirectory(${CMAKE_SOURCE_DIR}/test/matplot_test)
add_subdirectory(${CMAKE_SOURCE_DIR}/test/A_star_test)
add_subdirectory(${CMAKE_SOURCE_DIR}/test/Q_learning_test)
add_subdirectory(${CMAKE_SOURCE_DIR}/test/DWA_test)