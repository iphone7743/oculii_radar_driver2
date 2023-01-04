cmake_minimum_required(VERSION 3.0.2)
project(oculii_radar_driver)

set (pcl_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(catkin REQUIRED COMPONENTS
  image_transport
  pcl
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oculii_radar_driver
#  CATKIN_DEPENDS image_transport pcl pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


## Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

option(CANFD "Build canfd based sensor" OFF)
option(OPENCV "Build opencv related radar feature lib" ON)
option(BUILD_STATIC_LIBS "Build static library" OFF)
option(BUILD_DYNAMIC_LIBS "Build shared library" ON)

message("-----------------------")
message("-- CANFD=${CANFD}")
message("-- OPENCV=${OPENCV}")
message("-- BUILD_STATIC_LIBS=${BUILD_STATIC_LIBS}")
message("-- BUILD_DYNAMIC_LIBS=${BUILD_DYNAMIC_LIBS}")
message("-----------------------")


#####################
# PACKAGES/VARIABLES#
#####################
set(CMAKE_MACOSX_RPATH 0)

if(WIN32)

#Linux Compilation
else()

	#Set boost variable
	find_package(Boost REQUIRED COMPONENTS system filesystem)
	find_package(Threads REQUIRED)

	if(OPENCV)

		#Set opencv variable
		find_package(OpenCV REQUIRED)
		if(CANFD)
			#Set pcan variable
		 	set(PCAN_LIB_DIR /usr/include)
		  	set(PCAN_LIB /usr/lib/libpcanbasic.so)

			#Set OCULII_CORE_LIB variable
			set(OCULII_CORE_LIB
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/radar/canfd/libradar_shared.so
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/imu/libimu_shared.so
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/camera/libcamera_shared.so
			   )
		else()
			#Set OCULII_CORE_LIB variable
			set(OCULII_CORE_LIB
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/radar/ethernet/libradar_shared.so
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/imu/libimu_shared.so
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/camera/libcamera_shared.so
			   )
		endif(CANFD)	

	else()
		if(CANFD)
			#Set pcan variable
		 	set(PCAN_LIB_DIR /usr/include)
		  	set(PCAN_LIB /usr/lib/libpcanbasic.so)

			#Set OCULII_CORE_LIB variable
			set(OCULII_CORE_LIB
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/radar/canfd/libradar_shared.so
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/imu/libimu_shared.so
			   )
		else()
			#Set OCULII_CORE_LIB variable
			set(OCULII_CORE_LIB
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/radar/ethernet/libradar_shared.so
			    ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core/imu/libimu_shared.so
			   )
		endif(CANFD)	
	endif(OPENCV)

	#Set OCULII_SDK_LIB variable
	if(CANFD)
		set(OCULII_SDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_feature/canfd/liboculii_feat_shared.so)
	else()
		set(OCULII_SDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_feature/ethernet/liboculii_feat_shared.so)
	endif(CANFD)
	
        #Set OCULII_TRACKER_LIB variable
        set(OCULII_TRACKER_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/tracker/libFusionLibrary_shared.so)
endif(WIN32)


#Set OCULII_CORE_DEP variable
if(CANFD)
  if(OPENCV)
	  set(OCULII_CORE_DEP ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${PCAN_LIB})
  else()
	  set(OCULII_CORE_DEP ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${PCAN_LIB})
  endif(OPENCV)
else()
  if(OPENCV)
	  set(OCULII_CORE_DEP ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  else()
	  set(OCULII_CORE_DEP ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
  endif(OPENCV)
endif(CANFD)





#############
# INCLUDES #
#############

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/oculii_core/radar/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/oculii_core/imu/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/oculii_core/camera/include
)

if(OPENCV)
add_definitions(-DOPENCV=ON)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)
endif(OPENCV)

if(CANFD)
add_definitions(-DCANFD=ON)
include_directories(
  ${PCAN_LIB_DIR}
)
endif(CANFD)


#############
# LIBRARIES #
#############
# global
link_directories(${Boost_LIBRARIES_DIRS})
# oculii_core
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/win/oculii_core)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18/oculii_core)
# sdk_ws
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/win)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/ubuntu18)


############
# BINARIES #
############

link_libraries(${OCULII_SDK_LIB} ${OCULII_TRACKER_LIB} ${OCULII_CORE_LIB})
add_executable(oculii_driver src/main_driver.cpp)
target_link_libraries(oculii_driver ${OCULII_CORE_LIB} ${OCULII_TRACKER_LIB} ${OCULII_CORE_DEP} ${catkin_LIBRARIES})


unset(CANFD CACHE)
unset(OPENCV CACHE)
unset(BUILD_STATIC_LIBS CACHE)
unset(BUILD_DYNAMIC_LIBS CACHE)
