cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(segmentation)

set (PROJECT_VERSION 0.1)

set(CMAKE_BUILD_TYPE Release)
set(CXX_COMPILER_FLAGS -03 -fopenmp=libomp)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/include)

set(HEADERS
    ./include/segmentation/segmentation.hpp
    ./include/segmentation/config.h
    ./include/segmentation/segmentation_helpers.h
    ./include/segmentation/lsa_tr.h
    ./include/gco-v3.0/graph.h
    ./include/gco-v3.0/block.h
    ./include/gco-v3.0/energy.h
    ./include/gco-v3.0/GCoptimization.h
    ./include/gco-v3.0/LinkedBlockList.h   
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/gco-v3.0)

#add_library (maxflow gco-v3.0/maxflow.cpp)
#add_library (graph gco-v3.0/graph.cpp)
#add_library (LinkedBlockList gco-v3.0/LinkedBlockList.cpp)
#add_library (GCoptimization gco-v3.0/GCoptimization.cpp)

add_library (segmentation SHARED src/segmentation.cpp gco-v3.0/maxflow.cpp gco-v3.0/maxflow.cpp gco-v3.0/LinkedBlockList.cpp gco-v3.0/GCoptimization.cpp gco-v3.0/graph.cpp ${HEADERS})
target_link_libraries (segmentation ${PCL_LIBRARIES})

add_executable (segmentation_test src/segmentation_test.cpp)
target_link_libraries (segmentation_test ${PCL_LIBRARIES} segmentation)

#add_executable (segmentation_kinect_test src/segmentation_kinect_test.cpp)
#target_link_libraries (segmentation_kinect_test ${PCL_LIBRARIES} segmentation)

CONFIGURE_FILE(./src/segmentation.pc.in segmentation.pc @ONLY)

install(TARGETS segmentation segmentation_test
  RUNTIME DESTINATION bin 
  LIBRARY DESTINATION lib)

install(DIRECTORY include/segmentation/ DESTINATION include/segmentation FILES_MATCHING PATTERN "*.h")

install(DIRECTORY include/segmentation/ DESTINATION include/segmentation FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY include/gco-v3.0/ DESTINATION include/gco-v3.0 FILES_MATCHING PATTERN "*.h")


install(FILES ${CMAKE_CURRENT_BINARY_DIR}/segmentation.pc DESTINATION lib/pkgconfig)


