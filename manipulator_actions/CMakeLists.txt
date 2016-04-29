cmake_minimum_required(VERSION 2.8.3)
project(manipulator_actions)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs)

add_action_files(
    FILES
    ControlTorque.action
    ControlTable.action
)


generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    manipulator_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    manipulator_msgs
)
