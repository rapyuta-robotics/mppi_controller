cmake_minimum_required(VERSION 3.5)
project(mppi_controller)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-DXTENSOR_ENABLE_XSIMD)
add_definitions(-DXTENSOR_USE_XSIMD)

set(XTENSOR_USE_TBB 0)
set(XTENSOR_USE_OPENMP 0)
set(XTENSOR_USE_XSIMD 1)

# set(XTENSOR_DEFAULT_LAYOUT column_major)  # row_major, column_major
# set(XTENSOR_DEFAULT_TRAVERSAL row_major)  # row_major, column_major
find_package(catkin REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

set(dependencies_pkgs
  angles
  roscpp
  base_local_planner
  ddynamic_reconfigure
  dynamic_reconfigure
  costmap_2d
  geometry_msgs
  mbf_costmap_core
  mbf_msgs
  mbf_utility
  nav_core
  nav_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${dependencies_pkgs})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

generate_dynamic_reconfigure_options(
  cfg/MPPIController.cfg
  cfg/Ackermann.cfg
  cfg/NoiseGenerator.cfg
  cfg/GoalAngleCritic.cfg
  cfg/GoalCritic.cfg
  cfg/ObstacleCritic.cfg
  cfg/PathAlignCritic.cfg
  cfg/PathAngleCritic.cfg
  cfg/PathFollowCritic.cfg
  cfg/PreferForwardCritic.cfg
  cfg/TwirlingCritic.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${libraries}
  CATKIN_DEPENDS ${dependencies_pkgs}
)

add_library(mppi_controller SHARED
  src/controller.cpp
  src/optimizer.cpp
  src/critic_manager.cpp
  src/trajectory_visualizer.cpp
  src/path_handler.cpp
  src/noise_generator.cpp
)
add_library(mppi_critics SHARED
  src/critics/goal_critic.cpp
  src/critics/obstacles_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp

  # src/critics/path_align_legacy_critic.cpp
)

set(libraries mppi_critics mppi_controller)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  target_include_directories(${lib} PUBLIC ${xsimd_INCLUDE_DIRS}) # ${OpenMP_INCLUDE_DIRS}
  target_link_libraries(${lib} xtensor xtensor::optimize xtensor::use_xsimd ${catkin_LIBRARIES})
  add_dependencies(${lib} ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

install(TARGETS mppi_critics mppi_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES critics.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES mppic.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
