cmake_minimum_required(VERSION 2.8.3)
project(dragon_single_leg)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dragon_single_leg_lib
  CATKIN_DEPENDS
  DEPENDS system_lib Boost
)


include_directories(include ${catkin_INCLUDE_DIRS})

set(DDP_FILES  src/position_controller.cpp)
#set(DDP_FILES  src/velocity_controller.cpp)
add_library(dragon_single_leg_lib ${DDP_FILES} )

#add_executable(test src/test.cpp)

 target_link_libraries(dragon_single_leg_lib
  ${catkin_LIBRARIES}
  ${dragon_single_leg_lib_libraries}
 )
