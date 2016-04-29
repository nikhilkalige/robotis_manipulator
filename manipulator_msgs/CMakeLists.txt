cmake_minimum_required(VERSION 2.8.3)
project(manipulator_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(
    FILES
    ControlWrite.msg
    ControlTorque.msg
    PublishPosition.msg
    KeyValue.msg
)


generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
)
