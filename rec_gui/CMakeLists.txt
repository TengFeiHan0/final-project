cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
find_package(Qt4 COMPONENTS QtCore QtGui)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#uncomment if you have defined messages
rosbuild_genmsg()

# enable/disable some Qt features
#set( QT_USE_QTGUI TRUE )
#set( QT_USE_QTOPENGL TRUE )
#set( QT_USE_QTXML TRUE )


set(qt_srcs
	src/mainwindow.cpp
	src/listnerthr.cpp
	src/ros_thr.cpp
	src/CreatDataBuffer.cpp
	src/CreatBuffer.cpp
	src/pid_controller.cpp
	src/low_pass_filter.cpp
	#src/cam_pos_listner.cpp
	src/plot_publisher.cpp)
set(qt_hdrs
	src/mainwindow.h
	src/listnerthr.h
	src/ros_thr.h
	src/CreatDataBuffer.h
	src/HelperFunctions.h
	src/CreatBuffer.h
	src/pid_controller.h
	src/low_pass_filter.h
	#src/cam_pos_listner.h
	src/plot_publisher.h)
qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

QT4_WRAP_UI(uis_h src/mainwindow.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(qttest src/main.cpp
    ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(qttest ${QT_LIBRARIES})


#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
