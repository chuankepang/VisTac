cmake_minimum_required(VERSION 3.5)
project(traj_interpolator)

# 建议提升到 C++17，Eigen 和现代 ROS 2 配合更顺畅
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED) # Eigen3 首字母大写是标准做法

# 2. 添加可执行文件
add_executable(traj_interpolator src/interpolator_node.cpp)

# 3. 关联 ROS 依赖项
# 必须把所有用到的 ROS 包都写在这里，编译器才能找到对应的头文件
ament_target_dependencies(traj_interpolator
  rclcpp
  std_msgs
  tf2
  tf2_eigen
)

# 4. 关联 Eigen3 (Eigen 是头文件库，需要特殊链接方式)
target_link_libraries(traj_interpolator Eigen3::Eigen)

# 5. 安装配置
install(TARGETS traj_interpolator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()