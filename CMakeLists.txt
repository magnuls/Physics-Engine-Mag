cmake_minimum_required(VERSION 3.23)
project(3DEngineCpp)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# using cpp 20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#compiler flags
add_compile_options(-Wall -Wextra)

#getting the source files (own code under src/ + vendored stb_image under third_party/)
file(GLOB_RECURSE SOURCES
	src/*.cpp
	src/*.c
	third_party/stb/*.c)

#Remove the main function
list(REMOVE_ITEM SOURCES ${3DEngineCpp_SOURCE_DIR}/src/main.cpp)

#Create a library
add_library(engine STATIC ${SOURCES})

#Setting variable to the cmake directory
set(3DEngineCpp_CMAKE_DIR ${3DEngineCpp_SOURCE_DIR}/cmake)

# adding dependencies
find_package(OpenGL REQUIRED)
INCLUDE(${3DEngineCpp_CMAKE_DIR}/FindGLEW.cmake)
INCLUDE(${3DEngineCpp_CMAKE_DIR}/FindSDL2.cmake)
INCLUDE(${3DEngineCpp_CMAKE_DIR}/FindASSIMP.cmake)


# showing the headers to the compiler
target_include_directories(engine PUBLIC
    ${3DEngineCpp_SOURCE_DIR}/src              # your own headers
    ${3DEngineCpp_SOURCE_DIR}/third_party/stb  # vendored stb_image
    ${3DEngineCpp_SOURCE_DIR}/third_party/simd # vendored SIMD headers
    ${OPENGL_INCLUDE_DIRS}           # where GL/gl.h lives
    ${GLEW_INCLUDE_DIRS}             # where GL/glew.h lives
    ${SDL2_INCLUDE_DIRS}             # where SDL2/SDL.h lives
    ${ASSIMP_INCLUDE_DIRS}           # where assimp headers live
)

# showing linker where to link to 
target_link_libraries(engine PRIVATE
	${OPENGL_LIBRARIES}
	${GLEW_LIBRARIES}
	${SDL2_LIBRARIES}
	${ASSIMP_LIBRARIES}

)

#created the executable file and linked it to our engine library 
add_executable(3DEngineCpp ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_link_libraries(3DEngineCpp engine)

#Make sure you have Gtest
find_package(GTest REQUIRED)

#find the test files
file(GLOB_RECURSE TESTS
	${CMAKE_SOURCE_DIR}/tests/*.cpp
)

#allow testing
enable_testing()

# create executable
# NOTE: target is named `unit_tests`, NOT `testing`, on purpose: CTest writes
# its logs into a `Testing/` dir, which collides with a `testing` binary on
# case-insensitive filesystems (macOS/APFS) and breaks `ctest`. See §8 log.
add_executable(unit_tests ${TESTS})

# link lib + gtest to executable
target_link_libraries(unit_tests
	engine
	GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)

# Copy runtime assets into the build dir so ./3DEngineCpp finds ./res
# (non-destructive — replaces the old `mv ../res ./` gotcha).
file(COPY ${CMAKE_SOURCE_DIR}/res DESTINATION ${CMAKE_BINARY_DIR})









