#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from tacniq_entropy_visualize import read_tacniq_data

queue_len = 20
max_adc_value = 3500  # max adc value depends on FPGA
tac_map_height = 11
tac_map_width = 6
dim = tac_map_height * tac_map_width

left_queue = np.ones([queue_len, tac_map_height, tac_map_width]).astype(np.int16) * max_adc_value
right_queue = np.ones([queue_len, tac_map_height, tac_map_width]).astype(np.int16) * max_adc_value

left_queue_pub = rospy.Publisher('tacniq/queue_left', Int16MultiArray, queue_size=10)
right_queue_pub = rospy.Publisher('tacniq/queue_right', Int16MultiArray, queue_size=10)

def enqueue(queue, input_array):
    queue = np.roll(queue, 1, axis=0)
    queue[0] = input_array
    return queue

def get_left_tacniq_queue(msg): 
    global left_queue, left_queue_pub
    left_image = read_tacniq_data(msg)
    left_queue = enqueue(left_queue, left_image)

    # publish the data
    pub_msg = Int16MultiArray()
    # pub_msg.layout.dim.append(MultiArrayDimension())
    # pub_msg.layout.dim.append(MultiArrayDimension())
    # pub_msg.layout.dim[0].size = queue_len # 20
    # pub_msg.layout.dim[0].label = "time"
    # pub_msg.layout.dim[1].size = tac_map_height # 11
    # pub_msg.layout.dim[1].label = "height"
    # pub_msg.layout.dim[2].size = tac_map_width # 6
    # pub_msg.layout.dim[2].label = "width"
    # pub_msg.layout.dim[1].stride = dim # 66
    # pub_msg.layout.dim[2].stride = tac_map_width # 6
    
    pub_msg.data = left_queue.flatten().tolist()
    left_queue_pub.publish(pub_msg)
    # print(pub_msg.data)
    

def get_right_tacniq_queue(msg): 
    global right_queue, right_queue_pub
    right_image = read_tacniq_data(msg)
    right_queue = enqueue(right_queue, right_image)

    # publish the data
    pub_msg = Int16MultiArray()
    pub_msg.data = right_queue.flatten().tolist()
    right_queue_pub.publish(pub_msg)

if __name__ == '__main__':
    rospy.init_node("tacniq_visualizer")
    rospy.Subscriber('tacniq/left', Int16MultiArray, get_left_tacniq_queue)
    rospy.Subscriber('tacniq/right', Int16MultiArray, get_right_tacniq_queue)

    rospy.spin()
    