#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Int8MultiArray, MultiArrayDimension
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True



queue_len = 20
max_adc_value = 3500  # max adc value depends on FPGA
tac_map_height = 11
tac_map_width = 6
min_adc = 1500

max_std = 80

abs_tol = 3
std_tol = 0.25*0



left_image = np.ones([tac_map_height, tac_map_width])*(max_adc_value - min_adc)
right_image = np.ones([tac_map_height, tac_map_width])*(max_adc_value - min_adc)
left_std = np.zeros([tac_map_height, tac_map_width])
right_std = np.zeros([tac_map_height, tac_map_width])

contact_region_mask = np.zeros([tac_map_height, tac_map_width])
slip_region = np.zeros([tac_map_height, tac_map_width])

def read_left_tacniq(msg):
    global left_image
    left_image = read_tacniq_data(msg)
    left_image -= min_adc
    left_image = np.clip(left_image, 0, None)
    left_image = np.vectorize(transform)(left_image)


def read_right_tacniq(msg):
    global right_image
    right_image = read_tacniq_data(msg)
    right_image -= min_adc
    right_image = np.clip(right_image, 0, None)
    right_image = np.vectorize(transform)(right_image)

def read_left_std(msg):
    global left_std
    left_queue = read_queue(msg)
    left_std = np.std(left_queue, axis=0) 
    # print(left_std.max())

def read_right_std(msg):
    global right_std
    right_queue = read_queue(msg)
    right_std = np.std(right_queue, axis=0)
    # print(right_std.max())


def read_tacniq_data(msg):
    data_flattened = np.array(msg.data)
    height = msg.layout.dim[0].size
    width = msg.layout.dim[1].size
    data = data_flattened.reshape([height,width])
    return data

def read_queue(msg):
    data_flattened = np.array(msg.data)
    data = data_flattened.reshape([queue_len, tac_map_height, tac_map_width])
    return data

def transform(x, n=2):
    return int(2000-np.sqrt(2000**n - x**n))

def get_contact_mask(msg):
    global contact_region_mask

    contact_region_mask = np.array(msg.data).reshape(tac_map_height, tac_map_width)
    # print(contact_region_mask)

def get_slip_region(msg):
    global slip_region

    diff = np.array(msg.data).reshape(tac_map_height, tac_map_width)
    diff = diff * contact_region_mask

    contact_region_num = contact_region_mask.sum()
    diff_mean = 0
    diff_std = 0
    if contact_region_num > 0:
        diff_mean = diff.sum() / contact_region_mask.sum()
        diff_std = diff[contact_region_mask > 0].std()
    # print("mask", contact_region_num)
    # print("aaa", diff[contact_region_mask > 0].mean())
    # print("bbb", diff_mean)
    # print("std", diff_std)
    abs_mask = abs(diff) > abs_tol 
    std_mask = abs(diff - diff_mean) > std_tol * diff_std

    slip_region = (diff * abs_mask * std_mask)
# # ----------------------------------------------------------
# N = 50
# fps = 250
# frn = 75

# x = np.linspace(0, tac_map_width-1, tac_map_width).astype(np.int16)
# y = np.linspace(0, tac_map_height-1, tac_map_height).astype(np.int16)
# x, y = np.meshgrid(x, y)
# zarray = np.zeros((tac_map_width, tac_map_height, frn))

# f = lambda x, y, sig: 1 / np.sqrt(sig) * np.exp(-(x ** 2 + y ** 2) / sig ** 2)

# zarray = slip_region



# def change_plot(frame_number, zarray, plot):
#    plot[0].remove()
#    plot[0] = ax.plot_surface(x, y, zarray, cmap="afmhot_r")
# #    return all_artists

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# plot = [ax.plot_surface(x, y, zarray, color='0.75', rstride=1, cstride=1)]
# all_artists = []
# all_artists.append(plot[0])

# # ax.set_zlim(0, 1.1)
# # ani = animation.FuncAnimation(fig, change_plot, interval=0)

# # ax.axis('off')




# # anim = animation.FuncAnimation(fig, animate, init_func=init, interval=0, blit=True)
# ani = animation.FuncAnimation(fig, change_plot, fargs=(zarray, plot), interval=4, blit=True)
# plt.show()

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

N = 50
fps = 250
frn = 75

y = np.linspace(0, tac_map_width-1, tac_map_width).astype(np.int16)
x = np.linspace(0, tac_map_height-1, tac_map_height).astype(np.int16)
x, y = np.meshgrid(x, y)
zarray = np.zeros((tac_map_width, tac_map_height))

f = lambda x, y, sig: 1 / np.sqrt(sig) * np.exp(-(x ** 2 + y ** 2) / sig ** 2)

# for i in range(frn):
#    zarray[:, :, i] = f(x, y, 1.5 + np.sin(i * 2 * np.pi / frn))

def change_plot(frame_number, zarray, plot):
   plot[0].remove()
   plot[0] = ax.plot_surface(x, y, slip_region[x,y], cmap="afmhot_r")


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plot = [ax.plot_surface(x, y, zarray[:, :], color='0.75', rstride=1, cstride=1)]

ax.set_zlim(-50, 50)

rospy.init_node("tacniq_visualizer3D")
rospy.Subscriber('tacniq/left', Int16MultiArray, read_left_tacniq)
rospy.Subscriber('tacniq/right', Int16MultiArray, read_right_tacniq)
rospy.Subscriber('tacniq/queue_left', Int16MultiArray, read_left_std)
rospy.Subscriber('tacniq/queue_right', Int16MultiArray, read_right_std)
rospy.Subscriber('tacniq/diff_all', Float32MultiArray, get_slip_region)
rospy.Subscriber('/tacniq/mask_contact_region', Int8MultiArray, get_contact_mask)
ani = animation.FuncAnimation(fig, change_plot, frn, fargs=(zarray, plot), interval=1000 / fps)

# ax.axis('off')

plt.show()