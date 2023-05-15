#! /usr/bin/env python

import rospy
import franka_msgs
from franka_msgs.srv import SetForceTorqueCollisionBehavior, SetForceTorqueCollisionBehaviorRequest
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest
rospy.init_node('set_panda_defaults')

## TODO: CHECK THIS


# Set some sensible defaults
rospy.wait_for_service('/franka_control/set_force_torque_collision_behavior')
ftcb_srv = rospy.ServiceProxy('/franka_control/set_force_torque_collision_behavior', SetForceTorqueCollisionBehavior)
ftcb_msg = SetForceTorqueCollisionBehaviorRequest()
ftcb_msg.lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
ftcb_msg.upper_torque_thresholds_nominal = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
ftcb_msg.lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]  # These will be used by the velocity controller to stop movement
ftcb_msg.upper_force_thresholds_nominal = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

res = ftcb_srv.call(ftcb_msg).success
if not res:
    rospy.logerr('Failed to set Force/Torque Collision Behaviour Thresholds')
else:
    rospy.loginfo('Successfully set Force/Torque Collision Behaviour Thresholds')


gripper = rospy.get_param('~gripper')

rospy.wait_for_service('/franka_control/set_EE_frame') 
eef_srv = rospy.ServiceProxy('/franka_control/set_EE_frame', SetEEFrame)
eef_msg = SetEEFrameRequest()
if gripper == "panda":
    # 35mm down for the gripper
    gripper_offset = 0.035  + 0.10339999943971634
elif gripper == "robotiq":
    gripper_offset = 0.19660000056*0 + 0.244 # set 0 if the parameters are already configured in franka UI
    # rospy.logerr(f"{gripper_offset}\n"*10)
else:
    raise ValueError("Invalid gripper type")

# THIS IS NEW
# eef_msg.NE_T_EE = [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  gripper_offset + 0.10339999943971634, 1.0]
eef_msg.NE_T_EE = [0.707099974155426, -0.707099974155426, 0.0, 0.0, 0.707099974155426, 0.707099974155426, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  gripper_offset, 1.0]

res = eef_srv.call(eef_msg).success
if not res:
    rospy.logerr('Failed to set EE Frame')
else:
    rospy.loginfo('Successfully set EE Frame')