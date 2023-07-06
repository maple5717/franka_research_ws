#!/usr/bin/env python

# import rospy 
# from std_msgs.msg import Float64

# import time; time1 = None
# status_received = False 
# control_received = False

# def plot():
#     pass
# def control_callback(data):
#     pass 

# def state_callback(data):
#     pass

# if __name__ == "__main__":
#     rospy.init_node('pid_visualizer')

#     rospy.Subscriber('control_effort', Float64, control_callback)
#     rospy.Subscriber('state', Float64, state_callback)



import rospy
from std_msgs.msg import Float64
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation
import time
# matplotlib.use('agg')
time1 = None
status_received = False 
control_received = False

control_effort_list = []
state_list = []
time_list = []
time_control = []
time_status = []
def plot():
    global time_control, time_status, control_received, status_received

    if not(control_received and status_received):
        return 
    
    # if not time1:
    #     time1 = time.time()

    # time_elapsed = time.time() - time1
    
    # plt.clf()
    plt.plot(time_control, control_effort_list, label='control_effort')
    plt.plot(time_status, state_list, label='state')
    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('PID Visualizer')
    plt.grid(True)
    plt.show()

    # plt.tight_layout()
    control_received = False
    status_received = False
    # plt.pause(0.001)

def control_callback(data):
    global control_received

    time_control.append(time.time() - time1)
    control_received = True
    control_effort_list.append(data.data)
    plot()

def state_callback(data):
    global status_received
    time_status.append(time.time() - time1)
    status_received = True
    state_list.append(data.data)
    plot()

if __name__ == "__main__":
    rospy.init_node('pid_visualizer')

    plt.ion()
    time1 = time.time()
    rospy.Subscriber('control_effort', Float64, control_callback)
    rospy.Subscriber('state', Float64, state_callback)
    plt.show()
    rospy.spin()


    # while not (control_received and status_received):
    #     rospy.sleep(0.01)

    # while not rospy.is_shutdown():
    #     plot()
