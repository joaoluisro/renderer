cmake_minimum_required(VERSION 3.10)

# 1) Project name and language
project(renderer LANGUAGES CXX)

# 2) Require C++17 (or later)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# 3) Specify include directories
#    Assumes headers are in a top-level 'headers' directory
include_directories(${CMAKE_SOURCE_DIR}/headers)
# 4) Gather source files
#    Main is at root; other .cpp files live in 'source/'
set(SRC_FILES
    ${CMAKE_SOURCE_DIR}/main.cpp
)
file(GLOB ADDITIONAL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)
list(APPEND SRC_FILES ${ADDITIONAL_SOURCES})

# 5) Create the executable
add_executable(${PROJECT_NAME} ${SRC_FILES})

# 8) Link OpenMP
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

# 6) (Optional) Organize IDE views
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${SRC_FILES})

# 7) (Optional) Specify target include directories
#    Ensures headers directory is used when building this target
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/headers
    ${CMAKE_SOURCE_DIR}/libs
)
