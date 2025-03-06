# Program to convert rosbag data on topic IMU_data to csv (IMU message type)

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import csv

# Path for rosbag folder
bagpath = Path('/home/cooper530/Downloads/rosbag2_2025_03_06-14_02_43')

# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_JAZZY)

first_time = 0
firstTime = True

# Create csv file
with open('IMU_data.csv', 'w', newline='') as csvfile:
    # Create writer and header line
    writer = csv.writer(csvfile, delimiter=' ')
    writer.writerow(['time', 'orient_x', 'orient_y', 'orient_z', 'orient_w', 'ang_x', 'ang_y', 'ang_z', 'lin_x', 'lin_y', 'lin_z'])

    # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        # Specify topic here (since rosbag can subscribe to multiple topics), right now 'IMU_data'
        connections = [x for x in reader.connections if x.topic == 'IMU_data']
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            # Get the message components
            msg = reader.deserialize(rawdata, connection.msgtype)

            time_sec, time_nano = msg.header.stamp.sec, msg.header.stamp.nanosec
            orient_x, orient_y, orient_z, orient_w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            ang_x, ang_y, ang_z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            lin_x, lin_y, lin_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

            # If it's the first time, save this timestamp for future calculations
            if firstTime:
                first_time = (time_sec + time_nano * 1e-9)
                firstTime = False
            
            # Get the time starting from t = 0
            curr_time = (time_sec + time_nano * 1e-9) - first_time

            # Write row of info to csv
            writer.writerow([curr_time, orient_x, orient_y, orient_z, orient_w, ang_x, ang_y, ang_z, lin_x, lin_y, lin_z])