#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray, Int8MultiArray, MultiArrayDimension
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

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
    # print(diff.max())
    # print(slip_region)

def get_contact_mask(msg):
    global contact_region_mask

    contact_region_mask = np.array(msg.data).reshape(tac_map_height, tac_map_width)
    # print(contact_region_mask)

    

# left_image, right_image = get_data()

fig, ax = plt.subplots(nrows=2, ncols=4, figsize=(25, 25))  # initializing the figure
# print(ax)
# fig.canvas.set_window_title('Visualizer: Tactile Images')
fig.patch.set_facecolor('white')
cmap = 'Blues'
inverted_cmap = plt.cm.get_cmap(cmap).reversed()
# im = ax.imshow(right_image, vmin=0, vmax=max_adc_value, cmap=cmap)
im0 = ax[0, 0].imshow(left_image, vmin=0, vmax=(max_adc_value - min_adc), cmap=cmap)
im1 = ax[0, 1].imshow(right_image, vmin=0, vmax=(max_adc_value - min_adc), cmap=cmap)

im2 = ax[1, 0].imshow(left_std, vmin=0, vmax=max_std, cmap=inverted_cmap)
im3 = ax[1, 1].imshow(right_std, vmin=0, vmax=max_std, cmap=inverted_cmap)

im4 = ax[0, 2].imshow(np.abs(slip_region), vmax=60, cmap=inverted_cmap)
im5 = ax[0, 3].imshow(contact_region_mask, vmax=1, cmap=inverted_cmap)

y = np.linspace(0, tac_map_width-1, tac_map_width).astype(np.int16)
x = np.linspace(0, tac_map_height-1, tac_map_height).astype(np.int16)
X, Y = np.meshgrid(x, y)
# print(X, Y)
ax_surface = fig.add_subplot(2, 4, 7, projection='3d')
im6 = ax_surface.plot_surface(X, Y, (slip_region+10)[X, Y], cmap='viridis')

# ax[1, 2].clear()
# hist, bins = np.histogram(slip_region, bins=100, range=(-100, 100))
# ax[1, 2].bar(bins[:-1], hist, width=0.8)
n, bins, patches = ax[1, 2].hist(slip_region.flatten(), bins=100, range=(-100, 100))

# im6 = ax[1, 2].hist(slip_region, bins=100)

plt.colorbar(im0, ax=ax[0, 1])  # color legend
plt.colorbar(im3, ax=ax[1, 1])  # color legend
# plt.colorbar(im, ax=ax)  # color legend0

all_artists = []
all_artists.append(im0)
all_artists.append(im1)
all_artists.append(im2)
all_artists.append(im3)
all_artists.append(im4)
all_artists.append(im5)
all_artists.append(im6)
# all_artists.append(ax_surface)
# all_artists.append(im6)

ax[0, 0].set_xticks([])
ax[0, 0].set_yticks([])
ax[0, 0].set_title('Left', fontsize=30)
ax[0, 1].set_xticks([])
ax[0, 1].set_yticks([])
ax[0, 1].set_title('Right', fontsize=30)
ax[0, 2].set_xticks([])
ax[0, 2].set_yticks([])
ax[0, 2].set_title('Force Derivative', fontsize=30)
ax[0, 3].set_xticks([])
ax[0, 3].set_yticks([])
ax[0, 3].set_title('Contact Region', fontsize=30)


texts0 = [[None] * tac_map_height for _ in range(tac_map_width)]
texts1 = [[None] * tac_map_height for _ in range(tac_map_width)]

for x in range(tac_map_width):
    for y in range(tac_map_height):
        texts0[x][y] = ax[0, 0].text(x, y, '', fontsize=16, ha='center', va='center')
        texts1[x][y] = ax[0, 1].text(x, y, '', fontsize=16, ha='center', va='center')
        all_artists.append(texts0[x][y])
        all_artists.append(texts1[x][y])

def init():  # initialize the tactile map: ADC values
    global im0, im1, texts0, texts1, all_artists, left_image, right_image

    
    im0.set_array(left_image)
    im1.set_array(right_image)

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

def animate(i):  # this function is called sequentially
    global im0, im1, texts0, texts1, all_artists, left_image, right_image
    global left_std, right_std, slip_region
    
    mean = slip_region[contact_region_mask > 0].mean()
    mean = 0 if mean < 5 else mean 

    im0.set_array(left_image)
    im1.set_array(right_image)
    im2.set_array(left_std)
    im3.set_array(right_std)
    im4.set_array(np.abs(slip_region - mean))
    im5.set_array(contact_region_mask)

    print(slip_region[X, Y].ravel() )
    surface_updated = slip_region[X, Y].ravel() # np.sin(np.sqrt(X**2 + Y**2 + np.random.randn(1)[0])).ravel()
    # im6.set_zdata(surface_updated)
    im6._offsets3d = (X, Y, surface_updated)
    # print(slip_region[X, Y].ravel())

    
    # ax[1, 2].clear()
    hist, bins = np.histogram(slip_region[contact_region_mask > 0], bins=25, range=(-50, 50))
    # ax[1, 2].bar(bins[:-1], hist, width=0.8)
    # ax[1, 2].hist(slip_region.flatten(), bins=20, range=(-10, 10))
    # for count, rect in zip(hist, patches.patches):
    #     rect.set_height(count)
    print("EM Distance: ", np.abs(slip_region[contact_region_mask > 0]).mean())
    print("mean: ", mean )
    print("std: ", slip_region[contact_region_mask > 0].std())
    print("---------")
    for rect, h in zip(patches, hist):
        rect.set_height(h)
        print("|"+"*"*int(h))

    # ax[1, 2].set_xlabel('Value')
    # ax[1, 2].set_ylabel('Force Derivative')
    # ax[1, 2].set_title('Histogram')
    

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

if __name__ == '__main__':
    rospy.init_node("tacniq_visualizer")
    rospy.Subscriber('tacniq/left', Int16MultiArray, read_left_tacniq)
    rospy.Subscriber('tacniq/right', Int16MultiArray, read_right_tacniq)
    rospy.Subscriber('tacniq/queue_left', Int16MultiArray, read_left_std)
    rospy.Subscriber('tacniq/queue_right', Int16MultiArray, read_right_std)
    rospy.Subscriber('tacniq/diff_all', Float32MultiArray, get_slip_region)
    rospy.Subscriber('/tacniq/mask_contact_region', Int8MultiArray, get_contact_mask)
    anim = animation.FuncAnimation(fig, animate, init_func=init, interval=0, blit=True)
    plt.show()
    