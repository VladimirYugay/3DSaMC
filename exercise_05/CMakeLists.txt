cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(exercise_05)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")
set(glog_INCLUDE_DIR ${LIBRARY_DIR}/glog/include CACHE PATH "Path to glog source folder")
set(Ceres_DIR ${LIBRARY_DIR}/ceres/lib/cmake/Ceres CACHE PATH "Path to Ceres build folder")
set(Ceres_INCLUDE_DIR ${Ceres_DIR}/../include)
set(Flann_INCLUDE_DIR ${LIBRARY_DIR}/flann/build/src CACHE PATH "Path to Flann source folder")

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${Ceres_DIR})

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)

find_package(Ceres REQUIRED)
find_package(glog REQUIRED ${glog_DIR_HINT})

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
endif()

message(STATUS "ceres library name: ${Ceres_LIBRARIES}")
message(STATUS "path to ceres cmake information: ${Ceres_DIR}")
message(STATUS "path to ceres include directory: ${Ceres_INCLUDE_DIR}")
message(STATUS "glog library name: ${glog_LIBRARIES}")
message(STATUS "path to glog include directory = ${glog_INCLUDE_DIR}")
message(STATUS "path to flann include directory: ${Flann_INCLUDE_DIR}")


# Set files to be compiled
set(HEADER_FILES 
    Eigen.h 
    SimpleMesh.h 
    PointCloud.h 
    VirtualSensor.h 
    NearestNeighbor.h 
    ProcrustesAligner.h 
    ICPOptimizer.h 
    FreeImageHelper.h
)
set(SOURCE_FILES 
    FreeImageHelper.cpp
)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(exercise_05 main.cpp ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(exercise_05 flann flann_cpp lz4)
target_link_libraries(exercise_05 ceres glog::glog freeimage)
target_include_directories(exercise_05 PUBLIC ${Eigen3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}  ${Ceres_INCLUDE_DIR} ${glog_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR} ${Flann_INCLUDE_DIR})