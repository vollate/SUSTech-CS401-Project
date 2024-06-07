cmake_minimum_required(VERSION 3.0.2)
project(agilex_pure_pursuit)

#set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11 -o3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
)

include_directories(
  "/usr/include/eigen3"
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit 
  src/path.cpp 
  src/pure_pursuit.cpp
)

add_executable(record_path
  src/record_path.cpp
)

target_link_libraries(pure_pursuit
  ${catkin_LIBRARIES}
)

target_link_libraries(record_path
  ${catkin_LIBRARIES}
)

install(TARGETS pure_pursuit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
