cmake_minimum_required(VERSION 3.8)
project(cpp_keyence_plc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到 ROS 2 依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 創建一個共享庫 (library) `keyence_plc_com`
add_library(keyence_plc_com SHARED src/keyence_plc_com.cpp)
target_include_directories(keyence_plc_com PUBLIC include)
ament_target_dependencies(keyence_plc_com rclcpp)

# 創建 speed_test 並連結 keyence_plc_com
add_executable(speed_test src/speed_test.cpp)
target_link_libraries(speed_test keyence_plc_com)
ament_target_dependencies(speed_test rclcpp)

# 安裝共享庫 (libkeyence_plc_com.so)
install(
  TARGETS keyence_plc_com
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安裝 speed_test 可執行檔
install(TARGETS 
  keyence_plc_com
  speed_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
