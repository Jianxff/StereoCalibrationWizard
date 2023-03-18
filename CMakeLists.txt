set(PROJECT_NAME StereoCalibrationWizard)
project(${PROJECT_NAME})
cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ../../)

# add OpenCV library
set(OpenCV_DIR D:/.env/opencv4.6/build/x64/vc15/lib)
find_package( OpenCV REQUIRED )

# add source files
file(GLOB_RECURSE SOURCE_FILES "src/*")

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} )

message(STATUS "OpenCV Found in ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Source files includes ${SOURCE_FILES}")
