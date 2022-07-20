#!/usr/bin/env python3

import numpy as np
import rospy
import tf

rospy.init_node("create_tf")
br = tf.TransformBroadcaster()

i = 0

while not rospy.is_shutdown():
    br.sendTransform(
        (1 + i, 2 + i, 3 + i),
        tf.transformations.quaternion_from_euler(0, 0, i),
        rospy.Time.now(),
        "childframe",
        "world",
    )
    i = i + 1
    rospy.sleep(1.0)
