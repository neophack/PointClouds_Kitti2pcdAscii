cmake_minimum_required(VERSION 3.1)

project(kitti2pcd)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(Boost 1.60.0 COMPONENTS filesystem program_options)
# find_package(PCL 1.10 REQUIRED)  

include_directories(${Boost_INCLUDE_DIRS} )
link_directories(${Boost_LIBRARY_DIRS} )

# include_directories(${PCL_INCLUDE_DIRS} )
# link_directories(${PCL_LIBRARY_DIRS})

add_executable (kitti2pcd src/kitti2pcd.cpp)
target_link_libraries (kitti2pcd ${Boost_LIBRARIES})

add_executable (kitti2beam src/kitti2beam.cpp)
target_link_libraries (kitti2beam ${Boost_LIBRARIES})

add_executable (testdir src/testdir.cpp)
target_link_libraries (testdir ${Boost_LIBRARIES})

# add_executable (pcd2bin src/pcd2bin.cpp)
# target_link_libraries (pcd2bin  ${PCL_LIBRARIES})