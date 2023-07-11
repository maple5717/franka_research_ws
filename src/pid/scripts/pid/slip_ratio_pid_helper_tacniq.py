#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool, String, Int16MultiArray, Float32MultiArray, Int8MultiArray, MultiArrayDimension
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
import numpy as np
import threading
import ctypes
import inspect
from multiprocessing import Process, Event
from pid_helper_tacniq import PID_HELPER
# from math import abs

CONTACT_MODE = 0
SLIP_MODE = 1

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

def thread_job():
    print("start spinning")
    rospy.spin()

def async_raise(tid, exctype):
   """raises the exception, performs cleanup if needed"""
   tid = ctypes.c_long(tid)
   if not inspect.isclass(exctype):
      exctype = type(exctype)
   res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
   if res == 0:
      raise ValueError("invalid thread id")
   elif res != 1:
      # """if it returns a number greater than one, you're in trouble,  
      # and you should call it again with exc=NULL to revert the effect"""  
      ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
      raise SystemError("PyThreadState_SetAsyncExc failed")
      
def stop_thread(thread):
   async_raise(thread.ident, SystemExit)

class SLIP_PID_HELPER():
    def __init__(self, 
                 input_topic=rospy.get_param("~input", "input"), 
                 output_topic=rospy.get_param("~output", "output"),
                 goal=0.024*3*0 + 0.075, 
                 ):

        # tunable parameters
        self.TOLERANCE = 10
        self.TOLERANCE_QTY = 10
        self.TACNIQ_SCALE = 0.1
        self.ROBOTIQ_SCALE = 1
        self.abs_tol = 3
        self.std_tol = 0.25*0

        # define PID related parameters
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.GOAL = rospy.get_param("~goal", goal)
        self.state=0
        self.current_pos=0
        self.error = 1
        self.pid_enable = False

        # initialize the parameters
        self.contact_region_mask = np.zeros([tac_map_height, tac_map_width])
        self.slip_region = np.zeros([tac_map_height, tac_map_width])
        
        # define publishers
        self.pub = rospy.Publisher('state', Float64, queue_size=100)
        self.pub_goal = rospy.Publisher('setpoint', Float64, queue_size=100)
        self.pub_plant = rospy.Publisher(self.output_topic, outputMsg.Robotiq2FGripper_robot_output, queue_size=100)
        self.pub_pid_start = rospy.Publisher('pid_enable', Bool, queue_size=100)

        # define subsribers
        

        rospy.Subscriber(self.input_topic, inputMsg.Robotiq2FGripper_robot_input, self.getStatus)

        # command to be sent
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 0 #  1: activate the gripper, 0: reset the gripper -> try to activate the gripper from outside
        self.command.rGTO = 0 # Go To action: 0 or 1, 1 is action is taken
        self.command.rATR = 0 # Automatic Realease -> no need for now
        self.command.rPR = 0 # Desired target
        self.command.rSP = 0 # Desired speed: keep 0
        self.command.rFR = 0 # Desired force: keep 0
        
        self.init_gripper()
        self.pid_enable = False
        self.pub_pid_start.publish(Bool(data=0))
        # start with msg
        #rospy.Subscriber('talkPID', String, self.callbackPID)

    #def callbackPID(self, data):
    #    if data.data == 'start':
    #        self.pub_pid_start.publish(Bool(data=1))

    def getStatus(self, status):
        self.current_pos = status.gPO
        # self.updatePlant()
        #if self.current_pos >= self.GOAL:
        #    self.current_pos = self.GOAL

    def init_gripper(self):
        self.command.rACT = 0
        self.pub_plant.publish(self.command)
        rospy.sleep(0.2)
        self.command.rACT = 1
        self.command.rGTO = 1
        # self.command.rSP = 255
        # self.command.rFR = 150

        self.pub_plant.publish(self.command)
        self.command.rSP = 0
        print('Activated')

        # wait until open
        rospy.sleep(1.5)

        # send goal stuff
        # self.pub_goal.publish(Float64(data=self.GOAL))
        # print('Goal set')

    def updatePlant(self,data):
        '''receive and send output of the pid controller to gripper'''
        # if not(self.pid_enable):
        #     print(00000)
        #     return 
        # print(data.data)
        print("PID:", data.data)
        # action = self.current_pos + int(data.data*self.ROBOTIQ_SCALE) 
        # print('Input to the plant:', int(data.data*self.ROBOTIQ_SCALE) )
        # print('State: ', self.state)
        # print('Error: ', self.GOAL - self.state)
        # self.error = self.GOAL - self.state
        # # print("goal, state: ", self.GOAL, self.state, self.error)
        # action = max(0, action)
        # action = min(210, action)
        # self.command.rPR = action
        # # print('Pos + Action: ', self.current_pos, int(data.data*self.TACNIQ_SCALE), data.data)
        # # print('control effort: ', action)
        # # # print(self.current_pos, data.data, self.command.rPR)
        # self.pub_plant.publish(self.command)

    def get_contact_mask(self, msg):
        self.contact_region_mask = np.array(msg.data).reshape(tac_map_height, tac_map_width)

    def get_slip_region(self, msg):

        diff = np.array(msg.data).reshape(tac_map_height, tac_map_width)
        diff = diff * self.contact_region_mask

        contact_region_num = self.contact_region_mask.sum()
        diff_mean = 0
        diff_std = 0
        if contact_region_num > 0:
            diff_mean = diff.sum() / self.contact_region_mask.sum()
            diff_std = diff[self.contact_region_mask > 0].std()
        # print("mask", contact_region_num)
        # print("aaa", diff[contact_region_mask > 0].mean())
        # print("bbb", diff_mean)
        # print("std", diff_std)
        abs_mask = abs(diff) > self.abs_tol 
        std_mask = abs(diff - diff_mean) > self.std_tol * diff_std

        self.slip_region = (diff * abs_mask * std_mask)

        self.publish_slip_state()

    def publish_slip_state(self): 
        # combine left_data and right_data here and publish the result
        dist = 0
        taxel_in_contact = self.slip_region[self.contact_region_mask > 0]
        if taxel_in_contact.shape[0] > 0: 
            dist = np.abs(self.slip_region[self.contact_region_mask > 0]).mean()
        # print(self.TACNIQ_SCALE * dist)
        

        self.pub.publish(self.TACNIQ_SCALE * dist) # publish the combined data 
        self.state = self.TACNIQ_SCALE * dist # this is a temporary value (5 * average_force - 0*0.020768)
        # print(average_val, average_force)
        
        if self.pid_enable:
            self.pub_pid_start.publish(Bool(data=1))

    def listener(self): 
        # while self.command.rPR < 210:
        self.pub_goal.publish(Float64(data=self.GOAL))
        print('Goal set')

        self.pid_enable = True
        rospy.Subscriber('tacniq/diff_all', Float32MultiArray, self.get_slip_region)
        rospy.Subscriber('/tacniq/mask_contact_region', Int8MultiArray, self.get_contact_mask)

        rospy.Subscriber('control_effort', Float64, self.updatePlant)


        self.spin_thread = Process(target=thread_job) # threading.Thread(target=thread_job, daemon=True)
        self.spin_thread.start()

        last_command = self.command.rPR
        hold_flag = 0
        stop_flag = 0

        # stop grasping when error < 0.05 or is always zero
        while not(stop_flag):
            if abs(self.error) < 0.05 or (last_command == self.command.rPR): # last_command == self.command.rPR: #and self.error < 0.005: 
                if hold_flag:
                    stop_flag = 1
                    self.pid_enable = False
                    self.pub_pid_start.publish(Bool(data=0))

                    # self.spin_thread.terminate()
                    # self.spin_thread.join()
                    # stop_thread(self.spin_thread)
                    # print(self.spin_thread.is_alive)
                else:
                    hold_flag = 1
            

            last_command = self.command.rPR
            # print("hold", hold_flag)
            # print("stop", stop_flag)
            # print('Pos + Action: ', self.current_pos)
            # print('control effort: ', self.command.rPR)
            rospy.sleep(0.05)
            


if __name__ == '__main__':
    rospy.init_node('pid_helper')
    # initial_contact_helper = PID_HELPER()
    # rospy.sleep(0.1)
    # initial_contact_helper.listener()
    print("initial contact established")

    slip_controller_helper = SLIP_PID_HELPER(goal=0.1)

    rospy.sleep(0.1)
    slip_controller_helper.listener()
    print("finished grasping")

    # state is the output signal!!!!!!!!


    # remember to set the topic name of the C++ pid controller
