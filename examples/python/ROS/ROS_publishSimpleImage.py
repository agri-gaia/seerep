#!/usr/bin/env python3

# Note: This script must be started before the SEEREP client.
# This is due to the client subscribing to the in here created Node.

import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

rospy.init_node("create_image_rgb")
pub = rospy.Publisher("image", Image, queue_size=2)

rgb = []
lim = 256
for i in range(lim):
    for j in range(lim):
        x = float(i) / lim
        y = float(j) / lim
        z = float(j) / lim
        r = np.ubyte((x * 255.0) % 255)
        g = np.ubyte((y * 255.0) % 255)
        b = np.ubyte((z * 255.0) % 255)
        print(r, g, b)
        rgb.append(r)
        rgb.append(g)
        rgb.append(b)

header = Header()
header.frame_id = "map"

theImage = Image()
theImage.header = header
theImage.height = lim
theImage.width = lim
theImage.encoding = "rgb8"
theImage.step = 3 * lim
theImage.data = rgb

while not rospy.is_shutdown():
    theImage.header.stamp = rospy.Time.now()
    pub.publish(theImage)
    rospy.sleep(1.0)
