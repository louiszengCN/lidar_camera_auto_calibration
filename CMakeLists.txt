cmake_minimum_required(VERSION 3.16)
project(lidar-cam-calibration)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(project_pcd ./src/project.cpp)
target_link_libraries(project_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(gui_demo ./src/manual_calibration.cpp)
target_link_libraries(gui_demo ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Qt5::Widgets)

add_executable(depth_demo ./src/create_depth_pic.cpp)
target_link_libraries(depth_demo ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} Qt5::Widgets)


add_executable(auto_calib ./src/auto_calibration.cpp)
target_link_libraries(auto_calib ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} )
# target_include_directories(auto_calib PRIVATE /usr/local/include/ceres )

