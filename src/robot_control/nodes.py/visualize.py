#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
import matplotlib.pyplot as plt
from matplotlib import animation
from collections import deque

max_adc_value = 3500  # max adc value depends on FPGA
tac_map_height = 11
tac_map_width = 6


left_image = np.ones([tac_map_height, tac_map_width])*max_adc_value
right_image = np.ones([tac_map_height, tac_map_width])*max_adc_value
image_all = np.ones([66])*max_adc_value
# left_queue = deque(maxlen=1000)
# right_queue = deque(maxlen=1000)
taxel_queue = [deque(maxlen=1000)]*66

def read_left_tacniq(msg):
    global left_image, image_all, taxel_queue
    left_image = read_tacniq_data(msg)
    # left_queue.append(np.mean(left_image))
    # print(np.sum(left_image))

    image_all = (left_image[:, ::-1] + right_image) / 2
    image_all = image_all.reshape(-1)

    for i in range(66):
        taxel_queue[i].append(image_all[i])
    
def read_right_tacniq(msg):
    global right_image
    right_image = read_tacniq_data(msg)
    # right_queue.append(np.mean(right_image))

def read_tacniq_data(msg):
    data_flattened = np.array(msg.data)
    height = msg.layout.dim[0].size
    width = msg.layout.dim[1].size
    data = data_flattened.reshape([height,width])
    return data

# left_image, right_image = get_data()

fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(25, 25))  # initializing the figure
fig_plt = plt.figure()
ax_plt = plt.axes()
ax_plt.set_xlim(0, 1000)
ax_plt.set_ylim(max_adc_value-100, max_adc_value)

# fig.canvas.set_window_title('Visualizer: Tactile Images')
fig.patch.set_facecolor('white')
cmap = 'Blues'
# im = ax.imshow(right_image, vmin=0, vmax=max_adc_value, cmap=cmap)
im0 = ax[0].imshow(left_image, vmin=0, vmax=max_adc_value, cmap=cmap)
im1 = ax[1].imshow(right_image, vmin=0, vmax=max_adc_value, cmap=cmap)
# line1, = ax_plt.plot(np.array(left_queue))
# line2, = ax_plt.plot(np.array(right_queue))
# lines = [line1, line2]
lines = []
for index in range(66):
    lobj = ax_plt.plot([],[],lw=2)[0]
    lines.append(lobj)

# plt.colorbar(im0, ax=ax[1])  # color legend
# plt.colorbar(im, ax=ax)  # color legend0

all_artists = []
all_artists.append(im0)
all_artists.append(im1)

# print("aaa", left_queue)

ax[0].set_xticks([])
ax[0].set_yticks([])
ax[0].set_title('Left', fontsize=30)
ax[1].set_xticks([])
ax[1].set_yticks([])
ax[1].set_title('Right', fontsize=30)

texts0 = [[None] * tac_map_height for _ in range(tac_map_width)]
texts1 = [[None] * tac_map_height for _ in range(tac_map_width)]

for x in range(tac_map_width):
    for y in range(tac_map_height):
        texts0[x][y] = ax[0].text(x, y, '', fontsize=16, ha='center', va='center')
        texts1[x][y] = ax[1].text(x, y, '', fontsize=16, ha='center', va='center')
        all_artists.append(texts0[x][y])
        all_artists.append(texts1[x][y])

def init():  # initialize the tactile map: ADC values
    global im0, im1, texts0, texts1, all_artists, left_image, right_image

    
    im0.set_array(left_image)
    im1.set_array(right_image)

    # line1.set_data([])
    # line2.set_data([])
    

    for x in range(tac_map_width):
        for y in range(tac_map_height):
            # texts0[x][y].set_text(int(left_image[y,x]))
            if left_image[y,x] < 2500:
                texts0[x][y].set_color("black")
            else:
                texts0[x][y].set_color("white")

            # texts1[x][y].set_text(int(right_image[y,x]))
            if right_image[y,x] < 2500:
                texts1[x][y].set_color("black")
            else:
                texts1[x][y].set_color("white")
    return all_artists

def init_plt():
    global line1, line2, lines
    lines[0].set_data([], [])
    lines[1].set_data([], [])
    return lines

def animate(i):  # this function is called sequentially
    global im0, im1, texts0, texts1, all_artists, left_image, right_image
  
    im0.set_array(left_image)
    im1.set_array(right_image)
    # line1.set_data(np.array(left_queue))
    # line2.set_data(np.array(right_queue))
    for x in range(tac_map_width):
        for y in range(tac_map_height):
            # texts0[x][y].set_text(int(left_image[y,x]))
            if left_image[y,x] < 2500:
                texts0[x][y].set_color("black")
            else:
                texts0[x][y].set_color("white")

            # texts1[x][y].set_text(int(right_image[y,x]))
            if right_image[y,x] < 2500:
                texts1[x][y].set_color("black")
            else:
                texts1[x][y].set_color("white")
    return all_artists

def animate_plt(frame):
    # Update x1, y1, x2, y2 values
    # Example: x1 = ... ; y1 = ... ; x2 = ... ; y2 = ...
    x1 = np.arange(1000)
    y1 = np.random.randn(1000)
    global line1, line2, left_queue, right_queue
    if 1: #len(left_queue) * len(right_queue) > 0:
        # # plot mean value
        # lines[0].set_data(np.arange(len(left_queue)), np.array(left_queue).astype(np.float16))
        # lines[1].set_data(np.arange(len(right_queue)), np.array(right_queue).astype(np.float16))

        for i in range(66):
            lines[i].set_data(np.arange(len(taxel_queue[i])), np.array(taxel_queue[i]).astype(np.int16))
        # print(np.array(left_queue)[-1])
        # print(np.array(left_queue))
        # print(len(left_queue), np.array(left_queue).shape)
    return lines

if __name__ == '__main__':
    rospy.init_node("tacniq_visualizer")
    rospy.Subscriber('tacniq/left', Int16MultiArray, read_left_tacniq)
    rospy.Subscriber('tacniq/right', Int16MultiArray, read_right_tacniq)
    # anim = animation.FuncAnimation(fig, animate, init_func=init, interval=0, blit=True)
    # plt.show()
    anim2 = animation.FuncAnimation(fig_plt, animate_plt, init_func=init_plt, interval=0, blit=False)
    plt.show()