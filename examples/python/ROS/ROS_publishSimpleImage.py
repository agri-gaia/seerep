#!/usr/bin/env python3

"""
Test script to publish a random image to the '/image' topic.
"""

import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

PUB_FREQUENCY = 60
WIDTH = 1920
HEIGHT = 1080

rospy.init_node("rgb_image")
pub = rospy.Publisher("image", Image, queue_size=2)

header = Header()
header.frame_id = "map"

theImage = Image()
theImage.header = header
theImage.height = HEIGHT
theImage.width = WIDTH
theImage.encoding = "rgb8"
theImage.data = np.random.randint(0, 255, (HEIGHT, WIDTH, 3), dtype=np.uint8).tobytes()
theImage.step = 3 * WIDTH

rate = rospy.Rate(PUB_FREQUENCY)
while not rospy.is_shutdown():
    theImage.header.stamp = rospy.Time.now()
    pub.publish(theImage)
    rate.sleep()
