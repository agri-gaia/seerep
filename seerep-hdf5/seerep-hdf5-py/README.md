# seerep hdf5 python

Save geometries from python in hdf5 files

## Pointcloud example

```python
import seerep_hdf5_py as seerep
import numpy as np

# create file
file = seerep.File("testfile.h5")

# create seerep project (this step is mandatory to have a valid seerep hdf5 file)
file.createProject(project_name="seerep_project", root_frame_id="map")
file.setProjectGeolocation(coordinate_system="EPSG:4326",
                           ellipsoid="EPSG:6326",
                           latitude=52.356323,
                           longitude=8.280181,
                           altitude=0.0)

# create instance of pointcloud io module
pc_io = seerep.PointCloudIO(file)

# create dummy data
# labels
general_labels_crops = seerep.GeneralLabel(category="crops")
general_labels_crops.addLabel(seerep.InstanceLabel(label="label", instance_uuid="instance"))
general_labels_crops.addLabel(seerep.InstanceLabel(label="label1", instance_uuid="instance1"))

general_labels_weeds = seerep.GeneralLabel(category="weeds")
general_labels_weeds.addLabel(seerep.InstanceLabel(label="label2", instance_uuid="instance2"))

bb_labels = seerep.CategorizedBoundingBoxLabel3D(category="crops")
bb_labels.addLabel(seerep.BoundingBoxLabel3D(seerep.InstanceLabel(label="label3", instance_uuid="instance3"),
                                            min_point=[0, 0, 0],
                                            max_point=[1, 1, 2]))

# pointcloud data
# [[x, y, z], [x, y, z], ...]
points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
colors = np.array([[1, 1, 0.5], [1, 0, 1], [1, 1, 0], [1, 0, 1]], dtype=np.uint8)
other_field = np.array([[0.5], [0.1], [1.0], [3.141]], dtype=np.float32)
r = colors[:, 0]
g = colors[:, 1]
b = colors[:, 2]

# write pointcloud1 with points, colors and labels
pc_io.writePointCloud(uuid="cloud1",
                      frame_id="map",
                      seconds=0,
                      nanos=0,
                      sequence=0,
                      fields={
                        "xyz": points,
                        "rgb": colors
                      },
                      general_labels=[general_labels_crops, general_labels_weeds],
                      bb_labels=[bb_labels])

pc_io.writePointCloud(uuid="cloud2",
                      frame_id="map",
                      seconds=0,
                      nanos=0,
                      sequence=0,
                      fields={
                        "points": points,
                        "r": r,
                        "g": g,
                        "b": b,
                        "other": other_field
                      })

# get list of all available pointcloud uuids
cloud_ids = pc_io.getPointClouds()
print("cloud_ids", cloud_ids)

# load pointcloud "cloud1"
pointcloud_fields, general_labels, bb_labels = pc_io.readPointCloud(uuid="cloud1")
print("pointcloud_fields", pointcloud_fields)
print("general_labels", general_labels)
print("bb_labels", bb_labels)
```

## Image example

```python
import seerep_hdf5_py as seerep
import numpy as np
import cv2

# create file
file = seerep.File("testfile.h5")

# create seerep project (this step is mandatory to have a valid seerep hdf5 file)
file.createProject(project_name="seerep_project", root_frame_id="map")
file.setProjectGeolocation(coordinate_system="EPSG:4326",
                           ellipsoid="EPSG:6326",
                           latitude=52.356323,
                           longitude=8.280181,
                           altitude=0.0)

# create instance of image io module
img_io = seerep.ImageIO(file)

# generate labels
general_labels = seerep.GeneralLabel(category="crops")
general_labels.addLabel(seerep.InstanceLabel(label="label", instance_uuid="instance"))

bb_labels = seerep.CategorizedBoundingBoxLabel2D(category="crops")
bb_labels.addLabel(seerep.BoundingBoxLabel2D(seerep.InstanceLabel(label="label", instance_uuid="instance"),
                                             min_point=[0, 0],
                                             max_point=[1, 1]))

# load image
img = cv2.imread('TestImg.jpg')

# write image
img_io.writeImage(uuid="image1",
                  frame_id="map",
                  seconds=0,
                  nanos=0,
                  sequence=0,
                  image=img,
                  general_labels=[general_labels],
                  bb_labels=[bb_labels])

image_uuids = img_io.getImages()
print(image_uuids)

# read image
img, general_labels, bb_labels = img_io.readImage(uuid="image1")
cv2.imwrite('TestImgNew.jpg', img)
print("general_labels", general_labels)
print("bb_labels", bb_labels)
```

## Tf example

```python
import seerep_hdf5_py as seerep

# create file
file = seerep.File("testfile.h5")

# create seerep project (this step is mandatory to have a valid seerep hdf5 file)
file.createProject(project_name="seerep_project", root_frame_id="map")
file.setProjectGeolocation(coordinate_system="EPSG:4326",
                           ellipsoid="EPSG:6326",
                           latitude=52.356323,
                           longitude=8.280181,
                           altitude=0.0)

# create instance of tf io module
tf_io = seerep.TfIO(file)

# first transformation to store
tf1 = seerep.TfTransform()
tf1.frame_id = "map"
tf1.child_frame_id = "camera"
tf1.translation = [0, 0, 3] # x y z
tf1.rotation = [0, 0, 0, 1] # x y z w
tf1.seconds = 1676302069
tf1.nanos = 0

# second transformation to store
tf2 = seerep.TfTransform()
tf2.frame_id = "map"
tf2.child_frame_id = "camera"
tf2.translation = [0, 0, 3] # x y z
tf2.rotation = [0, 0, 0, 1] # x y z w
tf2.seconds = 1676302070
tf2.nanos = 1111111

# write transform twice to check (useful for multiple timestamps)
tf_io.writeTransform(tf1)
tf_io.writeTransform(tf2)

# get all tf entries
transformations = tf_io.readTransform("camera")
print("transforms", transformations)
```
