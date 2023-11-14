cmake_minimum_required(VERSION 3.1.0)
project(kyon_controller)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             sensor_msgs
             trajectory_msgs
             message_generation)

find_package(Eigen3 REQUIRED)

find_package(urdfdom REQUIRED)
find_package(pinocchio REQUIRED)
find_package(XBotInterface REQUIRED)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    ${XBotInterface_INCLUDE_DIRS})


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

add_executable(controller
    src/controller_node.cpp
    src/controller.cpp)

target_link_libraries(controller
    mpc_handler
    ${catkin_LIBRARIES})

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


