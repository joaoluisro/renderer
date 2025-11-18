cmake_minimum_required(VERSION 3.16)  # 3.12+ needed for CONFIGURE_DEPENDS
project(renderer LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type defaults
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# ---- File layout ----
# Re-scan automatically when files are added/removed (CONFIGURE_DEPENDS)
file(GLOB_RECURSE SRC_CPP CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/src/*.c"
)

file(GLOB_RECURSE HDR_ALL CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/src/*.h"
  "${CMAKE_SOURCE_DIR}/src/*.hpp"
  "${CMAKE_SOURCE_DIR}/headers/*.h"
  "${CMAKE_SOURCE_DIR}/headers/*.hpp"
)

set(SRC_FILES
  "${CMAKE_SOURCE_DIR}/main.cpp"
  ${SRC_CPP}
  ${HDR_ALL}     # listing headers makes Qt Creator show them
  ${UI_FILES}
  ${QRC_FILES}
)

# ---- Target ----
add_executable(${PROJECT_NAME} ${SRC_FILES})
message(STATUS "=== SRC_FILES ===")
foreach(f IN LISTS SRC_FILES)
  message(STATUS "  ${f}")
endforeach()

# Include paths
target_include_directories(${PROJECT_NAME} PRIVATE
  "${CMAKE_SOURCE_DIR}/headers"
  "${CMAKE_SOURCE_DIR}/libs"
)

# OpenMP (fix: actually call find_package)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX)
endif()

# Nice folders in IDEs (Qt Creator, VS, etc.)
source_group(TREE "${CMAKE_SOURCE_DIR}" FILES ${SRC_FILES})
message(STATUS "SRC_FILES=${SRC_FILES}")
