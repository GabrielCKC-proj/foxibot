cmake_minimum_required(VERSION 3.16)
project(foxibot_math)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(foxibot_common REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/trapezoidal_profile.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  foxibot_common::foxibot_common
  Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(foxibot_common Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_angles test_interpolation test_trapezoidal_profile)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_package()