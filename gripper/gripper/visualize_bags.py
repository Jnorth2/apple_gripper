#!/usr/bin/env python3

# This visualizes data from the bag files saved by user.py. 

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import matplotlib.pyplot as plt
import re

# Create a type store.
typestore = get_typestore(Stores.ROS2_FOXY)

#create arrays to plot
y_labels = ['Suction Cup 1 (hPa)', 'Suction Cup 2 (hPa)', 'Suction Cup 3 (hPa)', 'Distance Sensor (mm)', 'Motor Current (mA)', 'Motor Position (deg)', 'Motor Velocity (rpm)']
x_label = 'Time (s)'
sc1 = []
sc1_ts = []
sc2 = []
sc2_ts = []
sc3 = []
sc3_ts = []
dist = []
dist_ts = []
current = []
current_ts = []
position = []
position_ts = []
velocity = []
velocity_ts = []

topics = [sc1, sc2, sc3, dist, current, position, velocity]
timestamps = [sc1_ts, sc2_ts, sc3_ts, dist_ts, current_ts, position_ts, velocity_ts]

initial_time = 0

# create reader instance and open for reading - Edit this path to look at different bags.
with AnyReader([Path('apple_gripper/gripper/data/bags/20240717_1')], default_typestore=typestore) as reader:

    for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
        #collect initial time
        if initial_time == 0:
            initial_time = timestamp
        
        #read the message
        msg = reader.deserialize(rawdata, connection.msgtype)

        #extract tag and channel from message
        tag = re.search("\[Ch[0-9]*?\]", msg.data)
        channel = int(tag.group(0)[3:-1])

        #extract the data from message and add it to the lists
        data = re.search("(?:-*\d+)(\.*)(\d*)(?!.*\d)", msg.data)
        topics[channel].append(float(data.group(0)))

        #add the timestamps to the lists
        timestamps[channel].append((timestamp - initial_time) / 1000000000)



# plotting the points

fig, (ax1, ax2, ax3) = plt.subplots(3)
fig.suptitle('Motor Feedback Over the Apple Pick')
ax1.plot(timestamps[4], topics[4])
ax2.plot(timestamps[5], topics[5])
ax3.plot(timestamps[6], topics[6])

ax1.set(xlabel= x_label, ylabel=y_labels[4])
ax2.set(xlabel= x_label, ylabel=y_labels[5])
ax3.set(xlabel= x_label, ylabel=y_labels[6])


# plt.plot(timestamps[4], topics[4])

# # naming the x axis  
# plt.xlabel(x_label)  
# # naming the y axis  
# plt.ylabel(y_labels[4])  
# # giving a title to my graph  
# plt.title(y_labels[4] + ' Over the Apple Pick')

# function to show the plot
plt.show()

print(position)
