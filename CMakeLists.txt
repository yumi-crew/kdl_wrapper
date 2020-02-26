cmake_minimum_required(VERSION 3.5)
project(kdl_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(angles REQUIRED)

# kdl_wrapper
add_library(kdl_wrapper SHARED src/kdl_wrapper.cpp)
target_link_libraries(kdl_wrapper PRIVATE  ${orocos_kdl_LIBRARY_DIRS})
target_include_directories(kdl_wrapper PRIVATE include ${orocos_kdl_INCLUDE_DIRS})
ament_target_dependencies(kdl_wrapper
                          kdl_parser
                          urdf
                          rclcpp
                          orocos_kdl
)


install(DIRECTORY include/ DESTINATION include)

#libraries
install(TARGETS  kdl_wrapper
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(kdl_wrapper)
ament_package()