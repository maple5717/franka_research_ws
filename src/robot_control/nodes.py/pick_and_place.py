#! /usr/bin/env python3

# from __future__ import division, print_function

import rospy

# import os
# import time
# import datetime
# import numpy as np


# from std_msgs.msg import Int16
# from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
# import tf.transformations as tft

from franka_control_wrappers.panda_commander import PandaCommander
from pid.pid_helper_tacniq import PID_HELPER
# from mvp_grasping.utils import correct_grasp

# import dougsm_helpers.tf_helpers as tfh
from dougsm_helpers.ros_control import ControlSwitcher
import numpy as np

# from ggrasp.msg import Grasp

HOME_POSE = [-0.010087330820306798, -0.7763083389326214, 0.0073893375214589855, -2.354554671363115, -0.013121012074334023, 1.570351987068393, 0.78471674188274]
# EE_FRAME = "panda_hand_tcp"
EE_FRAME = "robotiq_eef"
# EE_FRAME = "panda_link8"

group_name = "panda_arm"
# group_name = "panda_robotiq"
pos_all = [
    [0.52, -0.20, 0.13-0.02, 1.0, 0.0, 0.0, 0.0], 
    [0.52, -0.00, 0.13-0.02, 1.0, 0.0, 0.0, 0.0], 
    [0.52, 0.20, 0.13-0.02, 1.0, 0.0, 0.0, 0.0], 
]

set_point_list = [3.8+0.5-0.6, 0.5, 1.2]
set_point_list = [3.8-1.5, 0.5, 1.2]
set_point_list = [2.3-0.8, 0.5+0.2, 1.1]
class MoveRobot(object):
    """
    Generic Move Group Class to control the robot.
    """

    def __init__(self, gripper):
        self.gripper = rospy.get_param("~gripper", gripper)

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher(
            "/cartesian_velocity_node_controller/cartesian_velocity",
            Twist,
            queue_size=1,
        )
        self.max_velo = 0.10
        self.curr_velo = Twist()

        self.cs = ControlSwitcher(
            {
                "moveit": "effort_joint_trajectory_controller",
                "velocity": "cartesian_velocity_node_controller",
            }
        )
        self.cs.switch_controller("moveit")
        self.pc = PandaCommander(group_name=group_name, gripper=self.gripper, ee=EE_FRAME)
        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        self.my_helper = None

        rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.__robot_state_callback,
            queue_size=1,
        )

    def __recover_robot_from_error(self):
        rospy.logerr("Recovering")
        self.pc.recover()
        self.cs.switch_controller("moveit")
        self.pc.goto_saved_pose("start", velocity=0.1)
        rospy.logerr("Done")
        self.ROBOT_ERROR_DETECTED = False

    def __robot_state_callback(self, msg):
        self.robot_state = msg
        if any(self.robot_state.cartesian_collision):
            if not self.ROBOT_ERROR_DETECTED:
                rospy.logerr("Detected Cartesian Collision")
            self.ROBOT_ERROR_DETECTED = True
        for s in FrankaErrors.__slots__:
            if getattr(msg.current_errors, s):
                self.stop()
                if not self.ROBOT_ERROR_DETECTED:
                    rospy.logerr("Robot Error Detected")
                self.ROBOT_ERROR_DETECTED = True


    def stop(self):
        self.pc.stop()
        self.curr_velo = Twist()
        self.curr_velo_pub.publish(self.curr_velo)


    def goto_home(self, velocity=1.0):
        return self.pc.goto_joints(HOME_POSE, velocity=velocity)
    
    def grasp(self, setpoint):
        if not self.my_helper:
            self.my_helper = PID_HELPER()
        
        self.my_helper.GOAL = setpoint
        rospy.sleep(0.1)
        self.my_helper.listener()
        print("finished grasping")
        # del my_helper
        

    def test(self):

        # print move group info
        self.pc.print_debug_info()
        self.cs.switch_controller("moveit")

        # go home pose
        print('Going home')
        self.goto_home(velocity=0.3)

        

        # ---start grasping---
        for object_pos, setpoint in zip(pos_all, set_point_list):
            object_pos_up = object_pos.copy()
            object_pos_up[2] += 0.05
            target_pos = object_pos.copy()
            target_pos[0] += 0.12
            target_pos_up = target_pos.copy()
            target_pos_up[2] += 0.05


            print('move to the object')
            pose = object_pos.copy()
            pose[2] += 0.1 # + 0.1m at z

            self.pc.goto_pose(object_pos_up, velocity=0.3)
            rospy.sleep(0.1)

            # go down
            print('go down')
            self.pc.goto_pose(object_pos, velocity=0.25)

            # close the fingers.
            print('Closing fingers')
            rospy.sleep(0.1)
            self.grasp(setpoint)
            print('successfully grasped ')

            print('go up')
            self.pc.goto_pose(object_pos_up, velocity=0.3)

            print('move to the target position')
            self.pc.goto_pose(target_pos_up, velocity=0.3)
            rospy.sleep(0.1)

            print('go down')
            self.pc.goto_pose(target_pos, velocity=0.25)

            # open
            print('Opening fingers')
            rospy.sleep(0.2)
            self.pc.gripper.set_gripper(0.1)

            print('go up')
            self.pc.goto_pose(target_pos_up, velocity=0.3)

        self.my_helper.spin_thread.terminate()
        self.my_helper.spin_thread.join()
        # go home 
        print('Going home pose')
        self.goto_home(velocity=0.2)

        return True
    
    def robotiq_freq_check(self):
        self.pc.gripper.set_gripper(0.1)

# def thread_job():
#     print("start spinning")
#     rospy.spin()
    
if __name__ == "__main__":
    rospy.init_node("move_robot")
    move_robot = MoveRobot(gripper="robotiq")

    if move_robot.test():
        print('Successfully executed all actions!')
        print('Done.')
        rospy.signal_shutdown('Shutting down the node')
        import sys; sys.exit()
        # raise Exception("end")
    # --------record frequency---------
    # print("start testing")
    # import numpy as np
    # num = 100

    # for _ in range(10):
    #     time_arr = np.zeros(num)
    #     for i in range(num):
    #         time1 = rospy.get_time()
    #         ret = move_robot.pc.gripper.read_state()
    #         time2 = rospy.get_time()

    #         time_arr[i] = time2 - time1
    #         # if ret:
    #         #     time_arr[i] = time2 - time1
    #         # else:
    #         #     time_arr[i] = 1

    #     print(f"average frequency: {np.mean(1/time_arr)} average time: {np.mean(time_arr)}  standard deviation: {np.std(time_arr)}")
    #     print(ret)





    # if move_robot.test():
    #     print('Successfully executed all actions!')
    #     print('Done.')
