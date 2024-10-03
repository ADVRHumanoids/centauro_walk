cmake_minimum_required(VERSION 3.1.0)
project(kyon_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             sensor_msgs
             trajectory_msgs
             message_generation
             tf
             tf_conversions)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(urdfdom REQUIRED)
find_package(pinocchio REQUIRED)
find_package(XBotInterface REQUIRED)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})


add_message_files(
    FILES
    WBTrajectory.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs geometry_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
    DEPENDS
    )

add_library(perception
    src/perception.cpp)

target_link_libraries(perception
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

install(TARGETS perception
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(controller
    src/controller_node.cpp
    src/controller.cpp)

target_link_libraries(controller
    mpc_handler
    perception
    ${catkin_LIBRARIES})

install(TARGETS controller
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(mpc_handler
    src/mpc_handler.cpp
    src/mpc_joint_handler.cpp
    src/resampler.cpp
)
target_link_libraries(mpc_handler
    yaml-cpp
    ${catkin_LIBRARIES}
    ${XBotInterface_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
    ${urdfdom_LIBRARIES}
)

install(TARGETS mpc_handler
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =============

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
    )

install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
    )

install(DIRECTORY python/ USE_SOURCE_PERMISSIONS
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/python
    FILES_MATCHING PATTERN "*.py"
    )
