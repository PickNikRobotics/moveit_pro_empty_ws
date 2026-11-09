cmake_minimum_required(VERSION 3.22)
project(agilex_ranger_mini_mock)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    description
    launch
    maps
    objectives
    params
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

# The vendored AgileX meshes are installed unmodified; the URDF loads them from here.
install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../external_dependencies/ugv_gazebo_sim/ranger_mini/ranger_mini_v3/meshes/
  DESTINATION
    share/${PROJECT_NAME}/meshes/agilex
)

install(
  PROGRAMS
    scripts/mock_lidar.py
    scripts/planar_base_bridge.py
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(amcl_tf_timing_test test/test_amcl_tf_timing.py TIMEOUT 60)
  ament_add_pytest_test(config_contract_test test/test_config_contract.py TIMEOUT 60)
  ament_add_pytest_test(scripts_test test/test_scripts.py TIMEOUT 60)
endif()

ament_package()
