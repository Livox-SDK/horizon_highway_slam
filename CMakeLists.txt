cmake_minimum_required(VERSION 2.8.3)
project(horizon_highway_slam)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Cholmod REQUIRED)

include_directories(
        include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
        ${CHOLMOD_INCLUDE_DIR}
)
find_library (HORIZON_LIB horizon ${PROJECT_SOURCE_DIR}/lib NO_DEFAULT_PATH)
link_libraries(${HORIZON_LIB})

find_library (G2O_LIB g2o ${PROJECT_SOURCE_DIR}/lib NO_DEFAULT_PATH)
link_libraries(${G2O_LIB})



catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS PCL
  INCLUDE_DIRS include
)

add_executable(horizon_highway_odometry src/horizon_highway_odometry.cpp)
target_link_libraries(horizon_highway_odometry
		${catkin_LIBRARIES} 
		${PCL_LIBRARIES} 
		${HORIZON_LIB} 
                ${G2O_LIB}
                cholmod
		${CHOLMOD_LIBRARIES}
		${BLAS_LIBRARIES}
		${LAPACK_LIBRARIES}
                )
add_executable (horizon_highway_mapping src/horizon_highway_mapping.cpp)
target_link_libraries(horizon_highway_mapping
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${HORIZON_LIB}
		cholmod
		${CHOLMOD_LIBRARIES}
		${BLAS_LIBRARIES}
		${LAPACK_LIBRARIES}
		)


