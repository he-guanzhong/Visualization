# cmake版本
cmake_minimum_required(VERSION 3.5.0)

# 程序版本
project(SPDPLAN VERSION 1.0)

# 添加include路径
include_directories(${PROJECT_SOURCE_DIR})

# 收集指定目录中所有源文件的名称，并将名称列表存储在coreFiles变量中
aux_source_directory(${PROJECT_SOURCE_DIR}/spdplanning coreFiles)
aux_source_directory(${PROJECT_SOURCE_DIR}/visualization coreFiles)
aux_source_directory(${PROJECT_SOURCE_DIR}/visualization/extension_package coreFiles)

# 库文件路径
# LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/reference)

# 编译可执行文件
add_executable(SpdPlan ${coreFiles})

# 编译静态库
# add_library(SpdPlan STATIC ${coreFiles})

# 切换速度规划回灌/融合/雷达显示模式
# target_compile_definitions(SpdPlan PRIVATE SPDPLAN_LOCAL_TEST)
# target_compile_definitions(SpdPlan PRIVATE AGSM_DEMO_TEST)
# target_compile_definitions(SpdPlan PRIVATE RADAR_DEMO_TEST)

# 编译过程中需要链接的文件
target_link_libraries(SpdPlan libeasyx.a)
