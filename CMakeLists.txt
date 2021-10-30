cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_CXX_STANDARD 14)

set( LVR_PROJECT_NAME lvr )
set( LVR_SOURCE_DIR src )
set( LVR_SOURCE ${LVR_SOURCE_DIR}/main.cpp )

project( ${LVR_PROJECT_NAME} )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( include )

# add_subdirectory( ${LVR_SOURCE_DIR} )

add_executable( 
    ${LVR_PROJECT_NAME}
    src/main.cpp 
    src/Engine.cpp 
    src/GestureClassifier.cpp
    src/GestureRecognizer.cpp
    src/Gesture
)
target_link_libraries( ${LVR_PROJECT_NAME} ${OpenCV_LIBS} )
