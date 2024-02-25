cmake_minimum_required(VERSION 3.0)
# project should be defined after setting compilers at the top level 
# to avoid infinite loop of configuration
project(filters VERSION 1.0.0 DESCRIPTION "filters to estimate the orientation")
message("Project = ${PROJECT_NAME}, path = ${CMAKE_CURRENT_LIST_DIR}")

# Set compilers:
set(CMAKE_C_COMPILER "C:/msys64/mingw64/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/msys64/mingw64/bin/g++.exe")
message("CMAKE_C_COMPILER = ${CMAKE_C_COMPILER}, CMAKE_CXX_COMPILER = ${CMAKE_CXX_COMPILER}")

# Set default build type:
if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
endif()
message("Build type = ${CMAKE_BUILD_TYPE}")

# Set standards:
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
message("C_STANDARD = ${CMAKE_C_STANDARD}")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
message("CXX_STANDARD = ${CMAKE_CXX_STANDARD}")

# Set compiler flag options
set(CMAKE_C_FLAGS "-Wall -Werror")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wmissing-braces -Wno-format -Wmissing-declarations -Waddress")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wpointer-arith -Wshadow -fno-strict-aliasing")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-misleading-indentation -Wunused-variable -Wno-unused-function -Wfloat-equal -g")

set(CMAKE_CXX_FLAGS "-MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable -Wno-unused-parameter -Wno-unused-function")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers -fno-inline -fno-common -g")

# Add source files of transform3d depending on options.
# Note: transform3d is necessary for filters.
file(GLOB TRANS_SRCS ${PROJECT_SOURCE_DIR}/transform3d/src/*.c)
set(TRANS_INCS ${PROJECT_SOURCE_DIR}/transform3d/inc)

# Add source files of filters depending on options.
# Default add all filters.
option(FILTERS_ALL "option for including all filters(defautlt)" OFF)
option(FILTERS_INTEGRAL "option for including integral filter" ON)
option(FILTERS_TILT "option for including tilt filter" ON)
option(FILTERS_COMPLEMENTARY "option for including complementary filter" ON)
option(FILTERS_MAHONY "option for including mahony filter" ON)
option(FILTERS_MADGWICK "option for including madgwick filter" ON)
if(FILTERS_ALL)
    set(FILTERS_INTEGRAL ON)
    set(FILTERS_TILT ON)
    set(FILTERS_COMPLEMENTARY ON)
    set(FILTERS_MAHONY ON)
    set(FILTERS_MADGWICK ON)
endif()

set(FILTERS_SRCS "")
set(FILTERS_INCS "")
if(FILTERS_INTEGRAL)
    file(GLOB INTEGRAL_SRCS ${CMAKE_CURRENT_LIST_DIR}/filters/integral/src/*.c)
    list(APPEND FILTERS_SRCS ${INTEGRAL_SRCS})
    list(APPEND FILTERS_INCS ${CMAKE_CURRENT_LIST_DIR}/filters/integral/inc)
    message("Integral filter added.")
endif()

if(FILTERS_TILT)
    file(GLOB TILT_SRCS ${CMAKE_CURRENT_LIST_DIR}/filters/tilt/src/*.c)
    list(APPEND FILTERS_SRCS ${TILT_SRCS})
    list(APPEND FILTERS_INCS ${CMAKE_CURRENT_LIST_DIR}/filters/tilt/inc)
    message("Tilt filter added.")
endif()

if(FILTERS_COMPLEMENTARY)
    file(GLOB COMPLEMENTARY_SRCS ${CMAKE_CURRENT_LIST_DIR}/filters/complementary/src/*.c)
    list(APPEND FILTERS_SRCS ${COMPLEMENTARY_SRCS})
    list(APPEND FILTERS_INCS ${CMAKE_CURRENT_LIST_DIR}/filters/complementary/inc)
    message("Complementary filter added.")
endif()

if(FILTERS_MAHONY)
    file(GLOB MAHONY_SRCS ${CMAKE_CURRENT_LIST_DIR}/filters/mahony/src/*.c)
    list(APPEND FILTERS_SRCS ${MAHONY_SRCS})
    list(APPEND FILTERS_INCS ${CMAKE_CURRENT_LIST_DIR}/filters/mahony/inc)
    message("Mahony filter added.")
endif()

if(FILTERS_MADGWICK)
    file(GLOB MADGWICK_SRCS ${CMAKE_CURRENT_LIST_DIR}/filters/madgwick/src/*.c)
    list(APPEND FILTERS_SRCS ${MADGWICK_SRCS})
    list(APPEND FILTERS_INCS ${CMAKE_CURRENT_LIST_DIR}/filters/madgwick/inc)
    message("Madgwick filter added.")
endif()

# Default we only build the dynamic library for Python wrapper.
# Build the dynamic library
option(FILTERS_BUILD_DYNAMIC "option for building filters dynamic library" ON)
if(FILTERS_BUILD_DYNAMIC)
    set(FILTERS_DYNAMIC_LIB "${PROJECT_NAME}_dynamic")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${filters_BINARY_DIR}/dll)
    add_library(${FILTERS_DYNAMIC_LIB} SHARED ${FILTERS_SRCS} ${TRANS_SRCS})
    # Include directory for header files
    target_include_directories(${FILTERS_DYNAMIC_LIB} PUBLIC ${FILTERS_INCS} ${TRANS_INCS})
endif()

# Build the static library
option(FILTERS_BUILD_STATIC "option for building filters static library" OFF)
if(FILTERS_BUILD_STATIC)
    set(FILTERS_STATIC_LIB "${PROJECT_NAME}_static")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${filters_BINARY_DIR}/lib)
    add_library(${FILTERS_STATIC_LIB} STATIC ${FILTERS_SRCS} ${TRANS_SRCS})
    target_include_directories(${FILTERS_STATIC_LIB} PUBLIC ${FILTERS_INCS} ${TRANS_INCS})
endif()

# Build the executable file
option(FILTERS_BUILD_EXE "option for building filters executable file" OFF)
if(FILTERS_BUILD_EXE)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${filters_BINARY_DIR}/bin)
    add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/main.c ${FILTERS_SRCS} ${TRANS_SRCS})
    target_include_directories(${PROJECT_NAME} PRIVATE ${FILTERS_INCS} ${TRANS_INCS})
endif()