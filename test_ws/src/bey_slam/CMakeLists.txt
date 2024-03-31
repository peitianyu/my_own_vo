cmake_minimum_required(VERSION 3.0.2)
project(bey_slam)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
)

# 添加eigen库
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/core
  src/test
)


# 运行测试
add_definitions(-DTEST_ENABLE)

file(GLOB_RECURSE ALL_LIBRIRY_SRCS "src/*.c*")

# 执行文件
add_executable(${PROJECT_NAME} ${ALL_LIBRIRY_SRCS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

