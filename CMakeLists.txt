cmake_minimum_required(VERSION 3.5)

project(DeadPixels LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(DeadPixels main.cpp)

include(GNUInstallDirs)
install(TARGETS DeadPixels
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( DeadPixels ${OpenCV_LIBS} )
