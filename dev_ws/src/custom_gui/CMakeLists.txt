cmake_minimum_required(VERSION 2.8.0)
project(custom_gui)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

find_package(catkin REQUIRED COMPONENTS rviz roscpp nav_msgs geometry_msgs sensor_msgs tf2 std_srvs message_generation)

find_package(Qt5 REQUIRED Core Widgets Multimedia Gui)
set(QT_LIBRARIES Qt5::Widgets Qt5::Multimedia Qt5::Gui)
include_directories(${catkin_INCLUDE_DIRS} ./include)

add_service_files(
  FILES
  Check.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(custom_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(custom_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS custom_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})