cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(playback)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



find_package(Eigen3 CONFIG REQUIRED)
find_package(PCL 1.2 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (ukf_highway src/main.cpp src/ukf.cpp src/tools.cpp src/render/render.cpp)
target_link_libraries (ukf_highway Eigen3::Eigen ${PCL_LIBRARIES})




