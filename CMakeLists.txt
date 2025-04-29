cmake_minimum_required(VERSION 3.16)
project(han_CPP)

add_library(han_CPP INTERFACE)

find_package(Eigen3 REQUIRED)
target_include_directories(han_CPP INTERFACE ${EIGEN3_INCLUDE_DIR})

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(han_CPP INTERFACE Python3::Python Python3::Module)

find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
        target_link_libraries(han_CPP INTERFACE Python3::NumPy)
else()
        target_compile_definitions(han_CPP INTERFACE WITHOUT_NUMPY)
endif()

install(TARGETS han_CPP EXPORT install_targets)

add_executable(main main.cpp)
target_link_libraries(main PRIVATE han_CPP)

add_subdirectory(PathPlanning bin_planning)