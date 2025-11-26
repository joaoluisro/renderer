cmake_minimum_required(VERSION 3.16)  # 3.12+ needed for CONFIGURE_DEPENDS
project(renderer LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

find_package(OpenMP REQUIRED)

set(LIBOMP_PREFIX "/opt/homebrew/opt/libomp")

# Help CMake/AppleClang find OpenMP
if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # # Tell CMake what flags & library to try
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "${LIBOMP_PREFIX}/lib/libomp.dylib")

    include_directories(SYSTEM "${LIBOMP_PREFIX}/include")
    link_directories("${LIBOMP_PREFIX}/lib")
endif()

target_link_libraries(renderer PRIVATE OpenMP::OpenMP_CXX)

# Nice folders in IDEs (Qt Creator, VS, etc.)
source_group(TREE "${CMAKE_SOURCE_DIR}" FILES ${SRC_FILES})
message(STATUS "SRC_FILES=${SRC_FILES}")
