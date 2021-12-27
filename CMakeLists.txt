cmake_minimum_required(VERSION 3.4)
project(NMPC)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(CASADI REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

include_directories(
   ${PROJECT_SOURCE_DIR}/include
   ${CASADI_INCLUDE_DIR}
   ${matplotlib_cpp_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/ModelCSTR.cpp
src/OptimalControlProblem.cpp
src/NonlinearModelPredictiveControl.cpp
src/Simulator.cpp
src/Plot.cpp
)

target_link_libraries(${PROJECT_NAME}
${CASADI_LIBRARIES}
yaml-cpp
Python3::Python
Python3::Module
Python3::NumPy
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/CSTR)
add_executable(nmpc_cstr Examples/CSTR/main_cstr.cpp)
target_link_libraries(nmpc_cstr ${PROJECT_NAME})
