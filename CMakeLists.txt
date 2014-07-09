cmake_minimum_required(VERSION 2.8.3)
project(moveit_python)

find_package(catkin)

catkin_package()
catkin_python_setup()

install(
  PROGRAMS
    scripts/dump_planning_scene.py
    scripts/list_objects.py
    scripts/load_planning_scene.py
    scripts/remove_objects.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
