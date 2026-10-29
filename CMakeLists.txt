cmake_minimum_required(VERSION 3.16)
project(drivecontrol LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

add_library(drivecontrol STATIC
  src/common.cpp
  src/path.cpp
  src/lateral.cpp
  src/longitudinal.cpp
  src/can_interface.cpp
  src/safety.cpp
  src/controller.cpp)
target_include_directories(drivecontrol PUBLIC include)

# Shared library with a C ABI for the Python simulator (ctypes). libstdc++
# is linked statically so the .so loads on hosts with a different toolchain.
add_library(drivecontrol_c SHARED src/capi.cpp)
target_link_libraries(drivecontrol_c PRIVATE drivecontrol -static-libstdc++ -static-libgcc)

add_executable(dc_replay tools/dc_replay.cpp)
target_link_libraries(dc_replay PRIVATE drivecontrol -static-libstdc++ -static-libgcc)

enable_testing()
foreach(t test_pid test_path_lateral test_longitudinal test_safety_can test_controller)
  add_executable(${t} tests/${t}.cpp)
  target_link_libraries(${t} PRIVATE drivecontrol)
  target_include_directories(${t} PRIVATE tests)
  add_test(NAME ${t} COMMAND ${t})
endforeach()
