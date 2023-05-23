cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.13.0
)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
  backward
  GIT_REPOSITORY https://github.com/bombela/backward-cpp
  GIT_TAG        v1.6
)
FetchContent_MakeAvailable(backward)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_library(${PROJECT_NAME}
  src/velo2cam.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${PCL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include/sensor_fusion
    ${PCL_INCLUDE_DIRS}
)

######## GTEST ########
add_executable(${PROJECT_NAME}_test
  gtest/test_main.cpp
  ${BACKWARD_ENABLE}
)

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  gtest
)
#####################

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_backward(${PROJECT_NAME}_test)

ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3 PCL)
ament_package()
