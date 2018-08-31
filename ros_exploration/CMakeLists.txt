cmake_minimum_required(VERSION 2.4.6)
set(OpenCV_DIR "/usr/local/opencv-2.4.9/share/OpenCV")
include_directories("/usr/local/opencv-2.4.9/include")
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8) 

include_directories( ... ${PCL_INCLUDE_DIRS})
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(Qt4 COMPONENTS QtCore QtGui)
find_package(OpenCV 2.4.9 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL 1.8 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})


rosbuild_init()
set(qt_srcs
	src/low_pass_filter.cpp
	src/pid_controller.cpp
	src/seg_fcn.cpp
	src/rrt.cpp
	src/potentialfield.cpp
	src/viewer.cpp
	src/listnerthr.cpp
	src/planning.cpp
	src/captureviewer.cpp
	src/frontier.cpp
	src/prm.cpp
	)
set(qt_hdrs
	include/low_pass_filter.h
	include/pid_controller.h
	include/seg_fcn.h
	include/rrt.h
	include/potentialfield.h
	include/viewer.h
	include/listnerthr.h
	include/planning.h
	include/CreatBuffer.h
	include/captureviewer.h
	include/frontier.h
	include/prm.h
	
)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

QT4_WRAP_UI(uis_h src/viewer.ui)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(ROS_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Debug)

rosbuild_add_executable(ros_exploration src/main.cpp ${uis_h} ${qt_srcs} ${qt_hdrs} ${qt_moc_srcs})

target_link_libraries(ros_exploration 
${QT_LIBRARIES}  
${PCL_LIBRARIES}  
${Boost_LIBRARIES}
${OpenCV_LIBRARIES}
libvtkCommon.so
libvtkFiltering.so libvtkWidgets.so libvtksys.so libQVTK.so libvtkQtChart.so libvtkViews.so
libvtkRendering.so libvtkGraphics.so libvtkImaging.so libvtkIO.so libvtkDICOMParser.so libvtkmetaio.so
libvtkexoIIc.so libvtkftgl.so libvtkHybrid.so
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
