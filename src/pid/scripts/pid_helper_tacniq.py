#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool, String, Int16MultiArray, MultiArrayDimension
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
import numpy as np

max_adc_value = 3500  # max adc value depends on FPGA
tac_map_height = 11
tac_map_width = 6


left_image = np.ones([tac_map_height, tac_map_width])*max_adc_value
right_image = np.ones([tac_map_height, tac_map_width])*max_adc_value
left_updated = False 
right_updated = False
both_updated = False
# reset and activate




def read_tacniq_data(msg):
    data_flattened = np.array(msg.data)
    height = msg.layout.dim[0].size
    width = msg.layout.dim[1].size
    data = data_flattened.reshape([height, width])
    return data




class PID_HELPER():
    def __init__(self):
        #+0.035 # 80 - 60 # in terms of desired pressure 230, 80
        self.TOLERANCE = 10
        self.TOLERANCE_QTY = 10

        self.input_topic = rospy.get_param("~input", "input")
        self.output_topic = rospy.get_param("~output", "output")
        self.GOAL = rospy.get_param("~goal", 0.024*3)
        self.state=0
        self.current_pos=0
        rospy.init_node('pid_helper')
        self.pub = rospy.Publisher('state', Float64, queue_size=100)
        self.pub_goal = rospy.Publisher('setpoint', Float64, queue_size=100)
        self.pub_plant = rospy.Publisher(self.output_topic, outputMsg.Robotiq2FGripper_robot_output, queue_size=100)
        self.pub_pid_start = rospy.Publisher('pid_enable', Bool, queue_size=100)
        rospy.Subscriber(self.input_topic, inputMsg.Robotiq2FGripper_robot_input, self.getStatus)

        # command to be sent
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 0 #  1: activate the gripper, 0: reset the gripper -> try to activate the gripper from outside
        self.command.rGTO = 0 # Go To action: 0 or 1, 1 is action is taken
        self.command.rATR = 0 # Automatic Realease -> no need for now
        self.command.rPR = 0 # Desired target
        self.command.rSP = 0 # Desired speed: keep 0
        self.command.rFR = 0 # Desired force: keep 0
        # print("aaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaaaaaaa")
        self.init_gripper()
        self.pub_pid_start.publish(Bool(data=0))
        # start with msg
        #rospy.Subscriber('talkPID', String, self.callbackPID)

    #def callbackPID(self, data):
    #    if data.data == 'start':
    #        self.pub_pid_start.publish(Bool(data=1))


    def getStatus(self, status):
        self.current_pos = status.gPO
        #if self.current_pos >= self.GOAL:
        #    self.current_pos = self.GOAL


    def init_gripper(self):
        self.command.rACT = 0
        self.pub_plant.publish(self.command)
        rospy.sleep(0.1)
        self.command.rACT = 1
        self.command.rGTO = 1

        self.pub_plant.publish(self.command)
        print('Activated')

        # wait until open
        rospy.sleep(2)

        # send goal stuff
        self.pub_goal.publish(Float64(data=self.GOAL))
        print('Goal set')


    # def updateState(self,data):
    #     self.state = data.bt_data[0].pdc_data

    #     if (abs(self.state - self.GOAL) < self.TOLERANCE) and (self.TOLERANCE_QTY != 0):
    #         #self.state = self.GOAL
    #         #self.pub_pid_start.publish(Bool(data=0)) 
    #         self.TOLERANCE_QTY -= 1
    #     if self.TOLERANCE_QTY == 0:
    #         self.pub_pid_start.publish(Bool(data=1))

    #     self.pub.publish(self.state)

    def updatePlant(self,data):
        '''receive and send output of the pid controller to gripper'''
        action = self.current_pos + int(data.data*200)
        print('Input to the plant:', data.data)
        print('State: ', self.state)
        print('Error: ', self.GOAL - self.state)
        action = max(0, action)
        action = min(210, action)
        self.command.rPR = action
        print('Pos + Action: ', self.current_pos, int(data.data*200), data.data)
        print('control effort: ', action)
        # print(self.current_pos, data.data, self.command.rPR)
        self.pub_plant.publish(self.command)

    def read_left_tacniq(self, msg):
        global left_image, both_updated, left_updated, right_updated
        left_image = read_tacniq_data(msg)
        self.publish_tacniq_state()
        left_updated = True
        # print(left_updated, right_updated)


    def read_right_tacniq(self, msg):
        global right_image, both_updated, left_updated, right_updated
        right_image = read_tacniq_data(msg)
        self.publish_tacniq_state()
        right_updated = True
        # print(left_updated, right_updated)

    def publish_tacniq_state(self):
        global left_data, right_data, left_updated, right_updated
        if left_updated and right_updated:
            # combine left_data and right_data here and publish the result
            average_val = np.mean(left_image) + np.mean(right_image)
            average_force = 1 - average_val/ 2 / max_adc_value
            self.pub.publish(5 * average_force-  0*0.020768) # publish the combined data 
            self.state = 5 * average_force - 0*0.020768 # this is a temporary value
            left_updated = False  # reset the flag
            right_updated = False
            self.pub_pid_start.publish(Bool(data=1))

    def listener(self): 
        rospy.Subscriber('tacniq/left', Int16MultiArray, self.read_left_tacniq)
        rospy.Subscriber('tacniq/right', Int16MultiArray, self.read_right_tacniq)
        rospy.Subscriber('control_effort', Float64, self.updatePlant)
        rospy.spin()

if __name__ == '__main__':
    my_helper = PID_HELPER()
    rospy.sleep(0.1)
    my_helper.listener()
