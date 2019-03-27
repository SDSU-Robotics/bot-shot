# Project's name
PROJECT ( BotShot C CXX )

# Specify the minimum version for CMake

cmake_minimum_required(VERSION 3.1)

add_definitions(-std=c++11)

# Set the output folder where your program will be created
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

find_package (Threads REQUIRED)
find_package(SDL2 REQUIRED)

# The following folder will be included
include_directories("${PROJECT_SOURCE_DIR}/include")

add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp ${PROJECT_SOURCE_DIR}/DriveBase.cpp
    ${PROJECT_SOURCE_DIR}/Controller.cpp ${PROJECT_SOURCE_DIR}/Display.cpp
    ${PROJECT_SOURCE_DIR}/Pickup.cpp ${PROJECT_SOURCE_DIR}/Launcher.cpp
    ${PROJECT_SOURCE_DIR}/Arduino.cpp ${PROJECT_SOURCE_DIR}/PIDController.cpp
    ${PROJECT_SOURCE_DIR}/PixyController.cpp )
    
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
else()
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(main ${CMAKE_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
endif()

target_link_libraries(main Threads::Threads)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
target_link_libraries(main ${SDL2_LIBRARIES})

# libpixyusb should always come before libboost and libusb #
target_link_libraries (main pixyusb)

find_package ( libpixyusb REQUIRED )
find_package ( libusb-1.0 REQUIRED )
find_package ( Boost 1.49 COMPONENTS thread system chrono REQUIRED)

target_link_libraries (main ${Boost_LIBRARIES})
target_link_libraries (main ${LIBUSB_1_LIBRARY})

file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

include_directories (src
                     include
                     ../../common
                     ${Boost_INCLUDE_DIR}
                     ${LIBUSB_1_INCLUDE_DIRS})
