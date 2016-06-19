from kdl_parser_py.urdf import treeFromParam as kdltree
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3
import rospy
import sys
import PyKDL


class Velocity:
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    def __init__(self):
        rospy.init_node('mainpulator_velocity', anonymous=True)
        (found, self.tree) = kdltree('robot_description')
        if not found:
            rospy.logfatal('manipulator_velocity', 'Unable to read robot description')
            sys.exit()

        prefix = rospy.get_namespace()
        prefix = prefix.strip('/') + '_'
        self.chain = self.tree.getChain(prefix + 'shoulder_link', prefix + 'ee_link')
        self.velocity_solver = PyKDL.ChainFkSolverVel_recursive(self.chain)
        self.JOINT_NAMES = [prefix + name for name in self.JOINT_NAMES]

        self.vel_pub = rospy.Publisher('end_effector_twist', Twist)

    def joint_state_callback(self, msg):
        velocity = PyKDL.JntArray(6)
        position = PyKDL.JntArray(6)
        idx = 0
        for joint in self.JOINT_NAMES:
            velocity[idx] = msg.velocity[msg.name.index(joint)]
            position[idx] = msg.position[msg.name.index(joint)]
            idx += 1

        end_frame = PyKDL.FrameVel()
        vel_array = PyKDL.JntArrayVel(position, velocity)

        self.velocity_solver.JntToCart(vel_array, end_frame)
        twist = end_frame.GetTwist()
        twist_msg = Twist(Vector3(*twist.vel), Vector3(*twist.rot))
        self.vel_pub.publish(twist_msg)

    def start(self):
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        rospy.spin()
