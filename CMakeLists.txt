cmake_minimum_required(VERSION 3.10)
project(luna_handeye_calibration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

foreach(script collect_apriltag solve_handeye camera_tf_broadcaster head_host_time capture_keyboard capture_preview)
  install(PROGRAMS luna_handeye_calibration/${script}.py
    DESTINATION lib/${PROJECT_NAME} RENAME ${script})
endforeach()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 固定使用 ROS 系统解释器；桌面 Conda 或用户 site 可能缺少 pytest，或加载不兼容插件。
  set(ENV{PYTHONNOUSERSITE} "1")
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(calibration_tests test
    PYTHON_EXECUTABLE /usr/bin/python3
    ENV PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()
ament_package()
