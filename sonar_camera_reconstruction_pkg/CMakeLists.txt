cmake_minimum_required(VERSION 3.14)
project(sonar_camera_reconstruction_pkg)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Python COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(sonar_oculus REQUIRED)
include_directories(${Python_INCLUDE_DIRS})

include_directories(/usr/include/eigen3)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c
          "import sysconfig; print(sysconfig.get_paths()['purelib'])"
  OUTPUT_VARIABLE PYTHON_INSTALL_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

pybind11_add_module(cfar
  sonar_camera_reconstruction_pkg/cpp_scripts/cfar.cpp
  ) 

# Install C++ module into Python path
install(TARGETS cfar
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

# Install Python code
ament_python_install_package(${PROJECT_NAME})

# Install entry point for the node
install(PROGRAMS
  ${PROJECT_NAME}/merge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
