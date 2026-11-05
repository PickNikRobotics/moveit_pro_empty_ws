cmake_minimum_required(VERSION 3.22)
project(agilex_ranger_mini_hw)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ranger_driver_bridge.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(hw_config_test test/test_hw_config.py TIMEOUT 60)
endif()

ament_package()
