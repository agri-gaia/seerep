# seerep hdf5 python

Save geometries from python in hdf5 files

## Pointcloud example

```python
import seerephdf5py as seerep
import numpy as np

file = seerep.File("testfile.h5")
pc_io = seerep.PointCloudIO(file)

points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
colors = np.array([[1, 1, 0.5], [1, 0, 1], [1, 1, 0], [1, 0, 1]], dtype=np.uint8)
r = colors[:, 0]
g = colors[:, 1]
b = colors[:, 2]

pc_io.writePointCloud("cloud1", "map", 0, 0, 0, { "xyz": points, "rgb": colors})
pc_io.writePointCloud("cloud2", "map", 0, 0, 0, { "xyz": points, "r": r, "g": g, "b": b})

pc_io.readPointCloud("cloud1")
```

## Image example

```python
import seerephdf5py as seerep
import cv2

file = seerep.File("testfile.h5")
img_io = seerep.ImageIO(file)

# generate labels
general_labels = seerep.GeneralLabel("crops")
general_labels.addLabel(seerep.InstanceLabel("label", "instance"))

bb_labels = seerep.CategorizedBoundingBoxLabel2D("crops")
bb_labels.addLabel(seerep.BoundingBoxLabel2D(seerep.InstanceLabel("label", "instance"), [0, 0], [1, 1]))

# load image
img = cv2.imread('TestImg.jpg')

# write image
img_io.writeImage("image1", "map", 0, 0, 0, img, general_labels=[general_labels], bb_labels=[bb_labels])

# read image
img_new, general_labels_new, bb_labels_new = img_io.readImage("image")

cv2.imwrite('TestImgNew.jpg', img_new)
```

## Tf example

```python
import seerephdf5py as seerep

file = seerep.File("testfile.h5")
tf_io = seerep.TfIO(file)

tf = seerep.TfTransform()
tf.frame_id = "a"
tf.child_frame_id = "b"
tf.translation = [0, 0, 3]
tf.rotation = [0, 0, 0, 1] # x y z w

# write transform twice to check (useful for multiple timestamps)
tf_io.writeTransform(tf)
tf_io.writeTransform(tf)

# get all tf entries
tf_io.readTransform("a")
```
