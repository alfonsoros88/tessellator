cmake_minimum_required (VERSION 2.8)

include (ExternalProject)

project (tessellator)

##  Install locally the RGBDReader to read an scene                           #

ExternalProject_Add (project_RGBDReader
    GIT_REPOSITORY "https://github.com/alfonsoros88/RGBDReader.git"
    GIT_TAG         master
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/contrib
)

include_directories (${CMAKE_BINARY_DIR}/contrib/include)

add_library(RGBDReader STATIC IMPORTED)
set_property(TARGET RGBDReader 
    PROPERTY IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/contrib/lib/libRGBDReader.so)

add_dependencies(RGBDReader project_RGBDReader)

##  Find the rest of the necessary libraries                                  #

find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS})

##  Build the executable

add_executable (tessellator src/main.cpp)
target_link_libraries (tessellator
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${Boost_LIBRARIES}
    boost_program_options
    boost_filesystem
    RGBDReader)

## Installation directives

install (TARGETS tessellator DESTINATION bin)
