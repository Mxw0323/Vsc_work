project(show_result)

# link_libraries(${EasyX_Lib_Dir}/libeasyx.a)

add_library(${PROJECT_NAME} 
SHARED
show_result.cpp
)

# 添加头文件路径
target_include_directories(${PROJECT_NAME}
PUBLIC
${EasyX_Inc_Dir}
)

# 链接静态库
target_link_libraries(${PROJECT_NAME} 
PRIVATE 
${EasyX_Lib_Dir}/libeasyx.a
)