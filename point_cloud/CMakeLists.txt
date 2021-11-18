cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lboost_thread")

set( LVR_PROJECT_NAME cloud )
set( LVR_SOURCE_DIR src )
set( LVR_SOURCE ${LVR_SOURCE_DIR}/main.cpp )

project( ${LVR_PROJECT_NAME} )

find_package( PCL REQUIRED )
find_package( Boost COMPONENTS thread REQUIRED )
# find_package( VTK REQUIRED )

include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( include )

link_directories( ${PCL_LIBRARY_DIRS} )

add_definitions( ${PCL_DEFINITIONS} )

add_executable( 
    ${LVR_PROJECT_NAME}
    src/main.cpp
    src/PLYLoader.cpp
    src/PCLVisualization.cpp
    src/EuclidianClusterSegmentation.cpp
    src/GroundExtractor.cpp
    src/Color.cpp
)
target_link_libraries( ${LVR_PROJECT_NAME} ${PCL_LIBRARIES} )
target_link_libraries( ${LVR_PROJECT_NAME} ${Boost_LIBRARIES} )