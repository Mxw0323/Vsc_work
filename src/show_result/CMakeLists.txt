project(show_result)

link_libraries(${EasyX_Lib_Dir}/libeasyx.a)

add_library(${PROJECT_NAME} 
SHARED
show_result.cpp
)


target_include_directories(${PROJECT_NAME}
PUBLIC
${EasyX_Inc_Dir}
)
