project(matplot_test)

add_executable(${PROJECT_NAME} matplot_test.cpp)

set(Matplot_Cpp_Dir ${CMAKE_SOURCE_DIR}/third_party/matplotlib-cpp-master)

target_include_directories(${PROJECT_NAME}
PUBLIC
${Matplot_Cpp_Dir}
${Python_Dir}
${Numpy_Dir}
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
${Python_Lib}
# ${Numpy_Lib}
)

add_test(
    NAME matplot_Cpp_test
    COMMAND ${PROJECT_NAME}
)