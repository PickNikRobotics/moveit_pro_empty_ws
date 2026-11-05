cmake_minimum_required(VERSION 3.22)
project(agilex_ranger_mini_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    mjcf
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/generate_mjcf.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_model_test test/test_sim_model.py TIMEOUT 120)
endif()

ament_package()
