#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

# def joint_states_callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
#     joint_positions = data.position
#     print("Gripper Joint Positions:", joint_positions)

# print("aaaa")
# rospy.init_node('joint_states_subscriber')
# rospy.Subscriber('/robotiq/joint_states', JointState, joint_states_callback)
# rospy.spin()



import robotiq_2f_gripper_control.baseRobotiq2FGripper
import robotiq_modbus_rtu.comModbusRtu
import time
import serial
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output, Robotiq2FGripper_robot_input
from robot_control.msg import RobotiqGripperState

def send_robotiq_joint_state(data):
    # maximum opening angle is 0.7854, source: x  
    # https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_e-Series_PDF_20190206.pdf
    gripper_opening = data.gPO * 0.7854 / 255 # (255 - data.gPO) * 0.085/255
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['finger_joint','left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_outer_knuckle_joint', 
                            'right_inner_knuckle_joint', 'right_inner_finger_joint']
    joint_state.position = [gripper_opening, 0.0, 0.0, 0.0, 0.0]
    pub.publish(joint_state)


rospy.init_node('robotiq_state_publisher')
rate = rospy.Rate(50)
pub = rospy.Publisher('/robotiq_state', JointState, queue_size=10)
# pub = rospy.Publisher('/robotiq_state', Robotiq2FGripper_robot_input, queue_size=10)
rospy.Subscriber("/input", Robotiq2FGripper_robot_input, send_robotiq_joint_state)

rospy.spin()
'''
device = "/dev/ttyUSB0"

gripper = (
    robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
)


# We connect to the address received as an argument
# gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()
# gripper.client.connectToDevice(device)
# while not(rospy.is_shutdown()):
#     status = gripper.getStatus()
#     msg = RobotiqGripperState()
#     msg.position = (255 - status.gPO) * 85/255
#     # print(status)
#     pub.publish(msg)
#     rate.sleep()





gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()
gripper.client.connectToDevice(device)
while True:
    time1 = time.time()
    # status = gripper.getStatus()
    print(gripper.client.getStatus(6))
    time2 = time.time()
    # print(status)
    # time.sleep(0.2)
    print("frequency: ", 1/(time2 - time1))
'''





