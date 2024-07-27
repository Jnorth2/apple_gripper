from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import matplotlib.pyplot as plt
import re

# Create a type store.
typestore = get_typestore(Stores.ROS2_FOXY)

#create arrays to plot
y_labels = [ 'Motor Current (mA)', 'Motor Position (deg)']
x_label = 'Time (s)'
current = []
current_ts = []
position = []
position_ts = []
topics = [current, position]
timestamps = [current_ts, position_ts]
initial_time = 0

# create reader instance and open for reading
with AnyReader([Path('desired_force_test')], default_typestore=typestore) as reader:
    for connection, timestamp, rawdata in reader.messages(connections=reader.connections):
        #collect initial time
        if initial_time == 0:
            initial_time = timestamp
        #read the message
        msg = reader.deserialize(rawdata, connection.msgtype)
        #extract tag and channel from message
        tag = re.search("\[Ch[0-9]*?\]", msg.data)
        channel = int(tag.group(0)[3:-1]) - 4
        #extract the data from message and add it to the lists
        data = re.search("(?:-*\d+)(\.*)(\d*)(?!.*\d)", msg.data)
        topics[channel].append(float(data.group(0)))
        #add the timestamps to the lists
        timestamps[channel].append((timestamp - initial_time) / 1000000000)

#align the data
cutoff1 = 200
cutoff2 = 300
cutoff3 = 252
cutoff4 = 395

#convert servo position to desired current using same code from arduino
desired_currents = []
for position in topics[1][cutoff1:cutoff2]:
    desired_current = 125 - (-2045.0 - position) / (-2045 + 2200) * 50
    if(desired_current>125):
        desired_current=125
    desired_currents.append(desired_current)


# make actual current positive for a nice graph
for i in range(len(topics[0])):
    topics[0][i] *= -1

# plotting the points
fig, (ax2, ax1) = plt.subplots(2)
plt.subplots_adjust(hspace=0)
fig.suptitle('Motor Feedback Over the Apple Grasp')

#current graph
ax1.plot(timestamps[1][cutoff1:cutoff2], desired_currents)
ax1.plot(timestamps[0][cutoff1:cutoff2], topics[0][cutoff1:cutoff2])
ax1.set(xlabel= 'Time (s)', ylabel=y_labels[0])
ax1.legend(["Desired Current ", "Real Current"], loc = 'lower right')

#force graph
data = []
time = []
with open('desired_force_test.log', "r") as file:
    for i, line in enumerate(file):
        if i>5:
            nums = line.split("\t")
            data.append(float(nums[1]))
            time.append(float(nums[2]))
ax2.plot(time[cutoff3:cutoff4], data[cutoff3:cutoff4], color = 'green')
ax2.set(ylabel='Fingertip Force (N)')
ax2.legend(["Fingertip Force"], loc = 'lower right')
ax2.set_xticks([])

plt.show()