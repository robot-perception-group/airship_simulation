cmake_minimum_required(VERSION 2.8.3)
project(nmpc_planner)


find_package(cmake_modules REQUIRED)
SET (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMakeModules)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  mav_msgs #This is for the firefly utility in gazebo, not telekyb
  generic_potential_field
  dynamic_reconfigure
  uav_msgs  
)


find_package(Boost QUIET REQUIRED COMPONENTS system thread)
#find_package(IPOPT REQUIRED)
find_package(Eigen REQUIRED)


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

generate_dynamic_reconfigure_options(
  cfg/nmpcPlannerParams.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
CATKIN_DEPENDS roscpp rospy
LIBRARIES nmpc_solver mpc_solver_SSRR mpc_planner_act_and_GT
)





INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${generic_potential_field_INCLUDE_DIRS})

# INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}  ${IPOPT_INCLUDE_DIRS} ${generic_potential_field_INCLUDE_DIRS})

add_library(nmpc_solver
  include/solver/MPCsolver.cpp
  include/solver/ldl.c
  include/solver/util.c
  include/solver/matrix_support.c
  include/solver/solver.c
)

add_library(mpc_solver_SSRR
	include/solver_obstacles_SSRR/MPCsolver.cpp
	include/solver_obstacles_SSRR/ldl.c
	include/solver_obstacles_SSRR/util.c
	include/solver_obstacles_SSRR/matrix_support.c
	include/solver_obstacles_SSRR/solver.c
)

#add_library(mpc_solver_act
#  include/solver_obstacles/MPCsolver.cpp
#  include/solver_obstacles/ldl.c
#  include/solver_obstacles/util.c
#  include/solver_obstacles/matrix_support.c
#  include/solver_obstacles/solver.c
#)

add_dependencies(nmpc_solver
        uav_msgs_generate_messages_cpp
)

add_dependencies(mpc_solver_SSRR
        uav_msgs_generate_messages_cpp
)

#add_dependencies(mpc_solver_act
#        uav_msgs_generate_messages_cpp
#)

target_link_libraries(nmpc_solver
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${generic_potential_field_LIBRARIES}
)

target_link_libraries(mpc_solver_SSRR
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${generic_potential_field_LIBRARIES}
)

#target_link_libraries(mpc_solver_act
#        ${Boost_LIBRARIES}
#        ${catkin_LIBRARIES}
#        ${generic_potential_field_LIBRARIES}
#)


add_executable(nmpc_planner src/nmpc_planner.cpp)
add_executable(mpc_SSRR src/mpc_SSRR.cpp)
add_executable(mpc_act src/mpc_act.cpp)
#add_executable(mpc_DQMPC src/mpc_DQMPC.cpp)

add_dependencies(nmpc_planner ${PROJECT_NAME}_gencfg)
add_dependencies(mpc_SSRR ${PROJECT_NAME}_gencfg)
add_dependencies(mpc_act ${PROJECT_NAME}_gencfg)
#add_dependencies(mpc_DQMPC ${PROJECT_NAME}_gencfg)

target_link_libraries(nmpc_planner ${catkin_LIBRARIES} nmpc_solver)
target_link_libraries(mpc_SSRR ${catkin_LIBRARIES} mpc_solver_SSRR)
target_link_libraries(mpc_act ${catkin_LIBRARIES} mpc_solver_SSRR)
#target_link_libraries(mpc_DQMPC ${catkin_LIBRARIES} mpc_solver_SSRR)
