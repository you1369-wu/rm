cmake_minimum_required(VERSION 3.22)
project(tracking_host_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)
add_executable(tracking_test tracking_test.cpp
  ../applications/tracking_mode/tracking_control/tracking_controller.cpp
  ../applications/tracking_mode/tracking_control/tracking_mode.cpp
  ../applications/tracking_mode/tracking_generation/trajectory_task.cpp
  ../sp_middleware/motor/rm_motor/rm_motor.cpp)
target_include_directories(tracking_test PRIVATE stubs ../applications ../sp_middleware)
target_link_libraries(tracking_test PRIVATE Threads::Threads)
if(MSVC)
  target_compile_options(tracking_test PRIVATE /UNDEBUG)
else()
  target_compile_options(tracking_test PRIVATE -UNDEBUG -Wall -Wextra -Werror)
endif()
enable_testing()
add_test(NAME tracking_behavior COMMAND tracking_test)
set_tests_properties(tracking_behavior PROPERTIES TIMEOUT 30)

add_executable(imu_support_test imu_support_test.cpp ../applications/imu/imu_support.cpp
  ../sp_middleware/io/bmi088/bmi088.cpp ../sp_middleware/tools/mahony/mahony.cpp)
target_include_directories(imu_support_test PRIVATE stubs ../applications ../sp_middleware)
target_compile_definitions(imu_support_test PRIVATE M_PI=3.14159265358979323846)
# SP's math header relies on a transitive size_t declaration on the embedded toolchain.
# Supply that standard declaration to host builds without changing the library.
if(MSVC)
  target_compile_options(imu_support_test PRIVATE /UNDEBUG /FIcstddef)
else()
  target_compile_options(imu_support_test PRIVATE -UNDEBUG -Wall -Werror -include cstddef)
endif()
add_test(NAME imu_support_behavior COMMAND imu_support_test)
set_tests_properties(imu_support_behavior PROPERTIES TIMEOUT 30)

add_executable(mode_integration_test yuntai_pid_integration_test.cpp
  ../applications/mission/mission.cpp
  ../applications/data_interfaces/can_task.cpp ../sp_middleware/tools/pid/pid.cpp
  ../sp_middleware/tools/math_tools/math_tools.cpp ../sp_middleware/motor/rm_motor/rm_motor.cpp)
target_include_directories(mode_integration_test PRIVATE fakes ../applications ../sp_middleware)
target_compile_definitions(mode_integration_test PRIVATE M_PI=3.14159265358979323846)
if(MSVC)
  target_compile_options(mode_integration_test PRIVATE /UNDEBUG /FIcstddef)
else()
  target_compile_options(mode_integration_test PRIVATE -UNDEBUG -include cstddef)
endif()
add_test(NAME mode_integration_behavior COMMAND mode_integration_test)
set_tests_properties(mode_integration_behavior PROPERTIES TIMEOUT 30)

add_executable(gravity_compensation_test gravity_compensation_test.cpp)
add_test(NAME gravity_compensation_behavior COMMAND gravity_compensation_test)

add_executable(mission_test mission_test.cpp ../applications/mission/mission.cpp)
target_include_directories(mission_test PRIVATE ../applications)
if(MSVC)
  target_compile_options(mission_test PRIVATE /UNDEBUG)
else()
  target_compile_options(mission_test PRIVATE -UNDEBUG -Wall -Wextra -Werror)
endif()
add_test(NAME mission_behavior COMMAND mission_test)
set_tests_properties(mission_behavior PROPERTIES TIMEOUT 30)
