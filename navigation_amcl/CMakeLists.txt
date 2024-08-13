cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(amcl LANGUAGES CXX C VERSION 1.0)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   # Mac OS X specific code
# SET(CMAKE_CXX_COMPILER "g++")
   SET(CMAKE_MACOSX_RPATH 1)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_subdirectory(src)
add_subdirectory(test)

