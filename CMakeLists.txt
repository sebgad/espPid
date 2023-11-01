if(EXISTS "../components")
  idf_component_register(SRCS ${SOURCES},
      INCLUDE_DIRS include, 
      REQUIRES esp_timer)
else()
cmake_minimum_required(VERSION 3.8)
  set(CMAKE_CXX_STANDARD 17)
  set(EXTRA_COMPONENT_DIRS include)
  include($ENV{IDF_PATH}/tools/cmake/project.cmake)
  project(espPid)
endif()
set(SOURCES espPid.cpp)
  
