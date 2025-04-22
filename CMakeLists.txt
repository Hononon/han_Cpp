cmake_minimum_required(VERSION 3.16)
project(han_CPP)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


#include(GNUInstallDirs)
set(PACKAGE_NAME han_CPP)


# Library target
add_library(han_CPP INTERFACE)


# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(han_CPP INTERFACE
        Python3::Python
        Python3::Module
        )
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
    target_link_libraries(han_CPP INTERFACE
            Python3::NumPy
            )
else()
    target_compile_definitions(han_CPP INTERFACE WITHOUT_NUMPY)
endif()
install(
        TARGETS han_CPP
        EXPORT install_targets
)



add_executable(main main.cpp)
target_link_libraries(main PRIVATE han_CPP)


# add_subdirectory(PathTracking bin)

add_subdirectory(PathPlanning bin_planning)