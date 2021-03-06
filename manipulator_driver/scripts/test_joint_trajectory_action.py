#! /usr/bin/env python
import roslib
import rospy
import actionlib
from manipulator_actions.msg import ControlTorqueGoal, ControlTorqueAction
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

roslib.load_manifest('manipulator_driver')
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
               'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
               'wrist_3_joint']


def get_current_joint_angles():
    msg = rospy.wait_for_message('joint_states', JointState)
    positions = []
    for joint in joint_names:
        positions.append(msg.position[msg.name.index(joint)])
    return positions


def torque_control(enable=True):
    client = actionlib.SimpleActionClient('control_torque', ControlTorqueAction)
    print("Waiting for torque server")
    client.wait_for_server()
    print("Connected to torque server")

    goal = ControlTorqueGoal()
    goal.joint_names = joint_names
    goal.enable = [enable] * len(joint_names)

    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    print result
    return result


def test_joint_trajectory_action():
    time_from_start = 0  # in seconds
    time_between_pts = 1
    pts_increment = -0.05
    goal_time_tolerance = rospy.Time(0.1)

    client = actionlib.SimpleActionClient(
        'pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    torque_control()

    def stop():
        client.cancel_goal()

    while not client.wait_for_server(rospy.Duration(5)):
        print("Waiting for joint trajectory action server")
    print("Connected to server")

    goal = FollowJointTrajectoryGoal()
    # rospy.on_shutdown(stop)

    goal.goal_time_tolerance = goal_time_tolerance
    goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                   'wrist_3_joint']
    # current_pos = get_current_joint_angles()
    # print current_pos
    current_pos = [
        0.002078120661087866, 1.6091038504393307, -0.8027179211422595,
        0.00599650479916318, -0.07228351752304148, -0.16890972463824888
    ]
    # First point is current position
    first_point = JointTrajectoryPoint()
    first_point.positions = current_pos[:]
    first_point.velocities = [0] * 6
    first_point.time_from_start = rospy.Duration(time_from_start)

    goal.trajectory.points.append(first_point)

    first_joint = current_pos[0]
    for pt in np.arange(first_joint, -.45, pts_increment):
        point = JointTrajectoryPoint()
        current_pos[0] = pt
        point.positions = current_pos[:]
        point.velocities = [0] * 6
        time_from_start = time_from_start + time_between_pts
        point.time_from_start = rospy.Duration(time_from_start)
        # print type(point.positions)
        goal.trajectory.points.append(point)

    # Start trajectory after 2 seconds
    # goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(2)
    # print goal
    print("Time secs", goal.trajectory.header.stamp.to_sec())
    client.send_goal(goal)
    print("Sent trajectory with %d points", len(goal.trajectory.points))
    for pt in goal.trajectory.points:
        print pt.positions
    client.wait_for_result()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Hooray, the arm finished the trajectory!")
    else:
        print("The arm failed to execute the trajectory.")
    print client.get_state()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('joint_trajectory_test_client', log_level=rospy.DEBUG)
        result = test_joint_trajectory_action()
        print result
        print("Finished execution")
    except rospy.ROSInterruptException:
        print("Program interrupted")
