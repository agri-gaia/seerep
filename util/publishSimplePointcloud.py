#!/usr/bin/env python3
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


rospy.init_node("create_cloud_xyzrgb")
pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)

points = []
lim = 8
for i in range(lim):
    for j in range(lim):
        for k in range(lim):
            x = float(i) / lim
            y = float(j) / lim
            z = float(k) / lim
            r = int(x * 255.0)
            g = int(y * 255.0)
            b = int(z * 255.0)
            a = 255
            print(r, g, b, a)
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
            print(hex(rgb))
            pt = [x, y, z, rgb]
            points.append(pt)

fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    # PointField('rgb', 12, PointField.UINT32, 1),
    PointField('rgba', 12, PointField.UINT32, 1),
]

print(points)

header = Header()
header.frame_id = "map"
pc2 = point_cloud2.create_cloud(header, fields, points)

while not rospy.is_shutdown():
    pc2.header.stamp = rospy.Time.now()
    pub.publish(pc2)
    rospy.sleep(1.0)
