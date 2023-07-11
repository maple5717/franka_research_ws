#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Int8MultiArray, MultiArrayDimension
from tacniq_entropy_visualize import read_tacniq_data
import cv2

queue_len = 20
max_adc_value = 3500  # max adc value depends on FPGA
tac_map_height = 11
tac_map_width = 6
alpha = 0.8 #0.9#5+0.05 # smoothing factor
dim = tac_map_height * tac_map_width

left_queue = np.ones([queue_len, tac_map_height, tac_map_width]).astype(np.int16) * max_adc_value
right_queue = np.ones([queue_len, tac_map_height, tac_map_width]).astype(np.int16) * max_adc_value

left_diff = np.zeros([tac_map_height, tac_map_width]).astype(np.int16)
right_diff= np.zeros([tac_map_height, tac_map_width]).astype(np.int16)

left_queue_pub = rospy.Publisher('tacniq/queue_left', Int16MultiArray, queue_size=10)
right_queue_pub = rospy.Publisher('tacniq/queue_right', Int16MultiArray, queue_size=10)

left_diff_pub = rospy.Publisher('tacniq/diff_left', Float32MultiArray, queue_size=10)
right_diff_pub = rospy.Publisher('tacniq/diff_right', Float32MultiArray, queue_size=10)
all_diff_pub = rospy.Publisher('tacniq/diff_all', Float32MultiArray, queue_size=10)

mask_contact_region_pub = rospy.Publisher('tacniq/mask_contact_region', Int8MultiArray, queue_size=10)

def enqueue(queue, input_array):
    queue = np.roll(queue, 1, axis=0)
    queue[0] = input_array
    return queue

def get_left_tacniq_queue(msg): 
    global left_queue, left_queue_pub, left_diff_pub, left_diff, all_diff_pub, right_queue
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

    # calculate the difference between the current tacniq output and 
    # that at the last time step
    current_diff = left_queue[0] - left_queue[1]
    left_diff = alpha * current_diff + (1 - alpha) * left_diff

    diff_pub_msg = Float32MultiArray()

    diff_all = left_diff[:, ::-1]+right_diff
    if np.max(np.abs(diff_all)) > 20 and False:
    #     print("left: \n", left_diff[:, ::-1])
    #     print("right: \n", right_diff)
        print("all: \n", diff_all.astype(np.int16))
    diff_pub_msg.data = left_diff.flatten().tolist()
    left_diff_pub.publish(diff_pub_msg)

    diff_pub_msg.data = diff_all.flatten().tolist()
    all_diff_pub.publish(diff_pub_msg)

    tacniq_average = (left_queue[0, :, ::-1] + right_queue[0, :])/2
    _, contact_region_mask = cv2.threshold(tacniq_average.flatten().astype(np.uint16), 0, 1, cv2.THRESH_BINARY +  cv2.THRESH_OTSU)
    contact_region_mask = 1 - contact_region_mask.astype(np.int8)

    mask_msg = Int8MultiArray()
    mask_msg.data = contact_region_mask.flatten().tolist()
    mask_contact_region_pub.publish(mask_msg)

    # print(pub_msg.data)
    

def get_right_tacniq_queue(msg): 
    global right_queue, right_queue_pub, right_diff_pub, right_diff
    right_image = read_tacniq_data(msg)
    right_queue = enqueue(right_queue, right_image)

    # publish the data
    pub_msg = Int16MultiArray()
    pub_msg.data = right_queue.flatten().tolist()
    right_queue_pub.publish(pub_msg)

    # calculate the difference between the current tacniq output and 
    # that at the last time step
    current_diff = right_queue[0] - right_queue[1]
    right_diff = alpha * current_diff + (1 - alpha) * right_diff

    diff_pub_msg = Float32MultiArray()
    # print("right: ", right_diff, "right")
    diff_pub_msg.data = right_diff.flatten().tolist()
    right_diff_pub.publish(diff_pub_msg)

if __name__ == '__main__':
    rospy.init_node("tacniq_visualizer")
    rospy.Subscriber('tacniq/left', Int16MultiArray, get_left_tacniq_queue)
    rospy.Subscriber('tacniq/right', Int16MultiArray, get_right_tacniq_queue)

    rospy.spin()
    