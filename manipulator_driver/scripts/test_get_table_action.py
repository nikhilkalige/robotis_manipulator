#! /usr/bin/env python
import roslib
import rospy
import actionlib
from manipulator_actions.msg import ControlTableAction, ControlTableGoal


roslib.load_manifest('manipulator_driver')


def control_torqure_action():
    client = actionlib.SimpleActionClient(
        '/leftarm/control_table', ControlTableAction)

    print("Waiting for the server")
    client.wait_for_server()
    print("Server Found")

    goal = ControlTableGoal()
    goal.joint_names = ['leftarm_shoulder_pan_joint', 'leftarm_shoulder_lift_joint', 'leftarm_elbow_joint', 'leftarm_wrist_2_joint']

    client.send_goal(goal)

    print("Send goal")
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('control_table_test_client')
        result = control_torqure_action()
        print result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
