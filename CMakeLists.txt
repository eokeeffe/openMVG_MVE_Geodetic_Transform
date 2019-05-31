cmake_minimum_required(VERSION 3.2)
project(pcl_utils)

# set the verison specifics
SET(APP_VERSION_MAJOR 0)
SET(APP_VERSION_MINOR 1)

# check compiler compatibility
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find libraries that will be required
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(RapidJSON)
find_package(PCL 1.5 REQUIRED)

# include headers and definitions
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# create programs from source files
add_executable (apply_transform src/transform.cpp)

# link programs to required libraries
target_link_libraries (apply_transform ${PCL_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    )

# No install directions, keep program runs local
