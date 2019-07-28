# Project's name
PROJECT (bot-shot)

# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(catkin REQUIRED)
find_package (Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories (src include ${catkin_INCLUDE_DIRS})

add_executable(Controller ${PROJECT_SOURCE_DIR}/src/Controller.cpp)
add_executable(DriveBase ${PROJECT_SOURCE_DIR}/src/DriveBase.cpp)
add_executable(Launcher ${PROJECT_SOURCE_DIR}/src/Launcher.cpp)
add_executable(Interface ${PROJECT_SOURCE_DIR}/src/Interface.cpp)

target_link_libraries(Controller ${catkin_LIBRARIES})
target_link_libraries(DriveBase ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(Launcher ${catkin_LIBRARIES} Threads::Threads)
target_link_libraries(Interface ${catkin_LIBRARIES})

if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi

    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_PhoenixCanutils.a)

    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/raspberry/libCTRE_PhoenixCanutils.a)

else()

    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(DriveBase /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_PhoenixCanutils.a)

    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(Launcher /home/robotics/catkin_ws/src/bot-shot/lib/x86-64/libCTRE_PhoenixCanutils.a)

endif()