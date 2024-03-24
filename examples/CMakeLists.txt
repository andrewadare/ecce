# Build all examples as executables

file(GLOB EXAMPLES *.cpp)

foreach(CPP_FILE ${EXAMPLES})
    cmake_path(GET CPP_FILE STEM EXENAME)
    add_executable(${EXENAME} ${CPP_FILE})
    target_link_libraries(${EXENAME} ${PROJECT_LIB})
endforeach()
