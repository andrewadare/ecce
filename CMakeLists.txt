cmake_minimum_required(VERSION 3.15)

project(ecce VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

find_package(OpenCV REQUIRED)
find_package(gtsam REQUIRED)

file(GLOB SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME}
    ${SOURCES}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    gtsam
)
