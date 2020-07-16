cmake_minimum_required(VERSION 2.8.3)
project(generic_potential_field)


find_package(catkin REQUIRED COMPONENTS roscpp  cmake_modules dynamic_reconfigure uav_msgs)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()

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
  cfg/potentialParams.cfg
)

INCLUDE_DIRECTORIES(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	src/CoTanPotentialFunctions.cpp
	src/PotentialFunction.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(calculus_test_new src/TestCalculus.cpp)

target_link_libraries(calculus_test_new
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

catkin_package(
    DEPENDS roscpp Boost
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

