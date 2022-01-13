cmake_minimum_required(VERSION 3.5)
execute_process(
    COMMAND bash "-c" "which nvcc" OUTPUT_VARIABLE CMAKE_CUDA_COMPILER)
project(ThermalSimulation LANGUAGES CXX CUDA)
enable_language(CUDA)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/*.cpp
                  ${PROJECT_SOURCE_DIR}/*.cu)

add_executable(${PROJECT_NAME} ${SOURCES})

# cuda
find_package(CUDA REQUIRED)
set(CMAKE_CUDA_ARCHITECTURES 52 60 61 75 CACHE STRING "CUDA architectures")
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_STANDARD 11)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --compiler-options -fno-strict-aliasing --ptxas-options=-v --generate-line-info")

include_directories(${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})
include_directories(/usr/local/cuda/include)
target_link_libraries(${PROJECT_NAME} -L/usr/local/cuda/lib64 -lcudart -lcuda)

# OpenCV
find_package(OpenCV 3.4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


