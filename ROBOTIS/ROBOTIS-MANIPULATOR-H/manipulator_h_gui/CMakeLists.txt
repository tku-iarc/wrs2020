################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.0)
project(manipulator_h_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  robotis_controller_msgs
  manipulator_h_base_module_msgs
  #qt_build
  cmake_modules
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_SRCS
  src/main.cpp
  src/main_window.cpp
  src/qnode.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/main_window.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

set(${PROJECT_NAME}_UIS
  ui/main_window.ui
)

set(${PROJECT_NAME}_RESOURCES
  resources/images.qrc
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

if(NOT EXISTS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
  file(MAKE_DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})
endif()

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robotis_controller_msgs manipulator_h_base_module_msgs cmake_modules eigen_conversions
  DEPENDS EIGEN3
)

qt5_wrap_cpp(${PROJECT_NAME}_MOCS     ${${PROJECT_NAME}_HDRS})
qt5_add_resources(${PROJECT_NAME}_RCC ${${PROJECT_NAME}_RESOURCES})

set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${manipulator_h_gui_INCLUDE_DIRECTORIES})

qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

################################################################################
# Build
################################################################################
include_directories(
  include
  ${manipulator_h_gui_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

################################################################################
# Build
################################################################################
# rosbuild_prepare_qt4(QtCore QtGui)

# file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
# file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/manipulator_h_gui/*.hpp)
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)

# QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
# QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})


#add_executable(manipulator_h_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(manipulator_h_gui ${manipulator_h_gui_SRCS} ${manipulator_h_gui_MOCS} ${manipulator_h_gui_UIS_H} ${manipulator_h_gui_RCC})
add_dependencies(manipulator_h_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(manipulator_h_gui ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Eigen3_LIBRARIES})
target_link_libraries(manipulator_h_gui Qt5::Widgets ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
################################################################################
# Install
################################################################################
install(TARGETS manipulator_h_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY resources ui
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
