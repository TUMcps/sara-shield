cmake_minimum_required(VERSION 3.16)

project(safety_shield VERSION 1.0.0 LANGUAGES CXX)

# === General Configuration ===
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# === Build Options ===
option(SAFETY_SHIELD_BUILD_TESTS "Build unit tests" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)

# === Debug Support ===
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND NOT ENABLE_COVERAGE)
  message(STATUS "Enabling debug flags (-g -O0)")
  add_compile_options(-g -O0)
endif()

# === Coverage Support ===
if(ENABLE_COVERAGE)
  message(STATUS "Enabling coverage build")

  # Global compile and link flags for coverage
  add_compile_options(--coverage -O0 -g -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)

  # Ensure LCOV and genhtml are available
  find_program(LCOV lcov REQUIRED)
  find_program(GENHTML genhtml REQUIRED)

  # Coverage target
  add_custom_target(coverage
    COMMAND ${LCOV} --directory . --capture --output-file coverage.info
                    --exclude '/usr/*' --exclude '*external/*' --exclude '*tests/*'
    COMMAND ${GENHTML} --demangle-cpp -o coverage coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating code coverage report"
  )
endif()

# === Dependencies ===
# Explicitly set the Eigen3 include directory path
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")

# Ensure Eigen3 is found using the explicit include directory
find_package(Eigen3 REQUIRED)

# Make Eigen available to all subprojects
include_directories(${EIGEN3_INCLUDE_DIR})

# GTest - optional
if(SAFETY_SHIELD_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  
  if(GTest_FOUND)
    include(GoogleTest)
    message(STATUS "GTest found. Building tests.")
  else()
    message(WARNING "GTest not found. Skipping tests. Install GTest or set SAFETY_SHIELD_BUILD_TESTS=OFF to suppress this warning.")
    set(SAFETY_SHIELD_BUILD_TESTS OFF)
  endif()
endif()

# Python bindings - optional
if(BUILD_PYTHON_BINDINGS)
  find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
  find_package(pybind11 QUIET)
  
  if(Python3_FOUND AND pybind11_FOUND)
    message(STATUS "Building Python bindings")
  else()
    message(WARNING "Python3 or pybind11 not found. Skipping Python bindings.")
    set(BUILD_PYTHON_BINDINGS OFF)
  endif()
endif()

# === External Dependencies ===
# Only add pybind11 if needed
if(BUILD_PYTHON_BINDINGS AND Python3_FOUND AND pybind11_FOUND)
  add_subdirectory(external/pybind11)
endif()

add_subdirectory(external/SaRA/ReachLib)
add_subdirectory(external/spdlog)
add_subdirectory(external/yaml-cpp)

# Ruckig dependency
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable Ruckig examples" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "Disable Ruckig Cloud Client" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "Disable Ruckig python module" FORCE)
add_subdirectory(external/ruckig)

# === Core Library ===
add_library(${PROJECT_NAME}
  src/safety_shield.cc
  src/long_term_traj.cc
  src/motion.cc
  src/path.cc
  src/robot_reach.cc
  src/human_reach.cc
  src/verify_iso.cc
  src/kalman_filter.cc
  src/measurement_handler.cc
  src/trajectory_utils.cc
  src/verification_utils.cc
  src/config_utils.cc
  src/planning_utils.cc
)

# Correctly specifying include directories for the target
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/SaRA/ReachLib/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/spdlog/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/yaml-cpp/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/ruckig/include>
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC SaRA  # PUBLIC so dependents inherit SaRA automatically
  PRIVATE spdlog yaml-cpp ruckig
)

# === Install Targets and Headers ===
install(TARGETS
  ${PROJECT_NAME}
  SaRA
  spdlog
  yaml-cpp
  ruckig
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/safety_shield DESTINATION include)
install(DIRECTORY external/SaRA/ReachLib/include/ DESTINATION include/SaRA)

# === Export CMake Targets ===
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE safety_shield::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/safety_shieldConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# === Optional: Pybind11 Python Bindings ===
if(BUILD_PYTHON_BINDINGS AND Python3_FOUND AND pybind11_FOUND)
  pybind11_add_module(safety_shield_py MODULE bindings/safety_shield_bindings.cpp)
  target_link_libraries(safety_shield_py PRIVATE ${PROJECT_NAME} spdlog yaml-cpp)
endif()

# === Optional: Debug Executables ===
foreach(exec debug self_constrained_collision_debug unconstrained_collision_debug hrgym_scenario_debug debug_real_world_setup ablation_conformal_prediction_sets_panda)
  add_executable(${exec} src/${exec}.cc)
  target_link_libraries(${exec} ${PROJECT_NAME} spdlog yaml-cpp)
endforeach()

add_executable(debug_ruckig src/debug_ruckig.cc)
target_include_directories(debug_ruckig PRIVATE
    include
    external/SaRA/ReachLib/include
    external/spdlog/include
    external/yaml-cpp/include
    external/ruckig/include
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(debug_ruckig
    ${PROJECT_NAME}
    spdlog
    yaml-cpp
    ruckig
)

# === GTest Unit Tests ===
if(SAFETY_SHIELD_BUILD_TESTS AND GTest_FOUND)
  foreach(test_file
    long_term_traj_tests
    human_reach_tests
    motion_tests
    path_tests
    robot_reach_tests
    verify_iso_tests
    safety_shield_tests
    trajectory_utils_tests
    verification_utils_tests
    planning_utils_tests)

    add_executable(${test_file} tests/src/${test_file}.cc)
    target_include_directories(${test_file} PUBLIC tests/include)

    target_link_libraries(${test_file}
      PRIVATE
      ${PROJECT_NAME}
      GTest::GTest
      GTest::Main
    )

    if(${test_file} STREQUAL "planning_utils_tests")
      target_link_libraries(${test_file} PRIVATE ruckig)
    endif()

    gtest_discover_tests(${test_file})
  endforeach()
endif()

configure_file(${CMAKE_SOURCE_DIR}/CTestCustom.cmake ${CMAKE_BINARY_DIR})