cmake_minimum_required(VERSION 3.1.0)
project(kyon_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             sensor_msgs
             message_generation)

find_package(Eigen3 REQUIRED)
find_package(cartesian_interface REQUIRED)

add_message_files(
    FILES
    SRBDCoordinate.msg
    SRBDTrajectory.msg
    FlushMeMaybe.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs geometry_msgs sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
    DEPENDS
    )

add_library(flush_me_maybe src/flush_me_maybe.cpp)
target_link_libraries(flush_me_maybe ${catkin_LIBRARIES})

add_executable(controller src/controller_node.cpp src/controller.cpp)
target_link_libraries(controller flush_me_maybe mpc_handler ${cartesian_interface_LIBRARIES} ${catkin_LIBRARIES})

add_library(mpc_handler
    src/mpc_handler.cpp
    src/mpc_joint_handler.cpp
    src/mpc_srbd_handler.cpp)
target_link_libraries(mpc_handler flush_me_maybe ${catkin_LIBRARIES}  ${cartesian_interface_LIBRARIES})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${muvt_core_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS})

install(TARGETS controller
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS flush_me_maybe
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS mpc_handler
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Install the launch directory
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    )



