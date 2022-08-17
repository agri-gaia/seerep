#!/usr/bin/env python3
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
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
