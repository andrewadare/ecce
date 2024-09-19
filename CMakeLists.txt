cmake_minimum_required(VERSION 3.20)

project(ecce VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)

# Build static lib from project sources
set(PROJECT_LIB ${PROJECT_NAME})
file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/*.?pp)
list(FILTER SOURCES EXCLUDE REGEX ".test.cpp")
add_library(${PROJECT_LIB} STATIC ${SOURCES})
include_directories(${PROJECT_LIB} SYSTEM PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_LIB}
    ${OpenCV_LIBS}
    gtsam
)

add_subdirectory(examples)
