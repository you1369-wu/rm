cmake_minimum_required(VERSION 3.22)
project(tracking_host_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_executable(tracking_test
  tracking_test.cpp
  ../applications/tracking_controller.cpp
  ../applications/tracking_mode.cpp
  ../applications/trajectory_task.cpp)
target_include_directories(tracking_test PRIVATE stubs ../applications)
if(MSVC)
  target_compile_options(tracking_test PRIVATE /UNDEBUG)
else()
  target_compile_options(tracking_test PRIVATE -UNDEBUG -Wall -Wextra -Werror)
endif()
enable_testing()
add_test(NAME tracking_behavior COMMAND tracking_test)
