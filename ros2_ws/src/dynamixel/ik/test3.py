#!/usr/bin/env python
from getch import getch
from ik.ik2 import ik_cal
import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class turtle_bot() :
    def __init__(self):
        rospy.init_node('turtle_bot', anonymous = True)

        self.pub = rospy.Publisher('/joint_cmd', JointTrajectory, queue_size=10)
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        ch = getch()
        self.joint_callback()
        

    def joint_callback(self):
        while 1 :
            char = getch()

            joint = JointTrajectory()
            joint.joint_names = ['Arm1', 'Arm2', 'Arm3', 'Arm4']

            pnt = JointTrajectoryPoint()

            while True:
                x, y = input("Enter input x, y: ").split()
                d = int(x)
                h = int(y)

                theta1, theta2, theta3 = ik_cal(d, h)

                if theta1 is None or theta2 is None or theta3 is None:
                    print("계산 실패. 다시 입력하세요.")
                    continue

                pnt.positions = [0, theta1, theta2, theta3]
                rospy.loginfo("Arms are moved")
                break
                
                            

            if char == chr(0x1b) : return
            joint.points.append(pnt)
            self.pub.publish(joint)
                

            rospy.loginfo("Arms are moved")



if __name__ == '__main__':
    controller = turtle_bot()
    rospy.spin()
