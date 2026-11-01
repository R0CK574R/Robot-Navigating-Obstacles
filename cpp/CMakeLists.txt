cmake_minimum_required(VERSION 3.20)
project(robotnav_cpp VERSION 1.0 LANGUAGES CXX
        DESCRIPTION "Robot obstacle navigation: the route-deciding core in C++")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)          # the core links into the Python module too
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# The core: the five planners, the hill-blind shortest route, D* Lite with a limited sensor, the air route.
add_library(robotnav_core STATIC
  src/map.cpp src/planners.cpp src/replan.cpp src/airspace.cpp src/mapio.cpp)
target_include_directories(robotnav_core PUBLIC include)
# Bit-for-bit agreement with Python and the browser: every + and * rounds on its own (no fused multiply-add, which
# compilers may otherwise use on ARM and on x86 with FMA), and nothing is reordered (no fast-math). PUBLIC, because
# the headers' inline code (edge costs, the D* Lite belief) is compiled into every target that uses them.
if(MSVC)
  target_compile_options(robotnav_core PUBLIC /fp:precise /W4)
else()
  target_compile_options(robotnav_core PUBLIC -ffp-contract=off -fno-fast-math -Wall -Wextra)
endif()

# robotnav: the command-line tool (route, paths, explore, fly, bench on a map file from `python main.py export`).
add_executable(robotnav src/cli.cpp)
target_link_libraries(robotnav PRIVATE robotnav_core)

# Unit tests on small hand-made maps (ctest). The comparison with Python on the real maps is tests/test_cpp.py.
enable_testing()
add_executable(robotnav_tests tests/test_core.cpp)
target_link_libraries(robotnav_tests PRIVATE robotnav_core)
add_test(NAME core COMMAND robotnav_tests)

# robotnav_cpp: the Python module (pybind11), so Python can run the C++ core and check it against its own.
option(ROBOTNAV_PYTHON "Build the robotnav_cpp Python module (needs pybind11: pip install pybind11)" ON)
if(ROBOTNAV_PYTHON)
  find_package(Python 3.9 COMPONENTS Interpreter Development.Module)
  if(Python_FOUND AND NOT pybind11_DIR)
    execute_process(COMMAND "${Python_EXECUTABLE}" -m pybind11 --cmakedir
                    OUTPUT_VARIABLE pybind11_DIR OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  endif()
  find_package(pybind11 CONFIG QUIET)
  if(pybind11_FOUND)
    pybind11_add_module(robotnav_cpp bindings/module.cpp)
    target_link_libraries(robotnav_cpp PRIVATE robotnav_core)
    message(STATUS "Building the robotnav_cpp Python module for ${Python_EXECUTABLE}")
  else()
    message(STATUS "pybind11 not found: skipping the Python module (pip install pybind11, then configure again)")
  endif()
endif()
