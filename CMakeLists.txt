cmake_minimum_required(VERSION 3.10)
project(test)

file(GLOB NUMBER_SYSTEM_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# PCL
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Sophus REQUIRED)

add_library(NumberSystem STATIC ${NUMBER_SYSTEM_SRC})
target_include_directories(NumberSystem PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(NumberSystem)

add_executable(test_squad test_squad.cpp)
target_link_libraries(test_squad
                      NumberSystem
                      ${PCL_LIBRARIES}
                      Eigen3::Eigen)

add_executable(helloSO3 helloSO3.cpp)
target_link_libraries(helloSO3
                      Sophus::Sophus)

add_executable(helloso3 helloso3.cpp)
target_link_libraries(helloso3
                      Sophus::Sophus
                      ${PCL_LIBRARIES})


