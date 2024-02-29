#!/usr/bin/env python3

import cv2
import numpy as np

root = ["/seerep/seerep-data/simulatedData/lighting_01/", "/seerep/seerep-data/simulatedData/lighting_02/"]

labelSwitch = {
    1.0: "corn",
    2.0: "http://aims.fao.org/aos/agrovoc/c_820",
    3.0: "clovers",
    4.0: "daisy",
    5.0: "curly dock",
}

color = (255, 255, 255)

for folderIndex in range(2):
    imagePath = root[folderIndex] + "camera_main_camera/rect/"
    outputPath = root[folderIndex] + "camera_main_camera/withAnnotation/"
    annotationPath = root[folderIndex] + "camera_main_camera_annotations/bounding_box/"

    for i in range(16):
        baseFilePath = imagePath + str(i).zfill(4)
        baseOutputPath = outputPath + str(i).zfill(4)
        baseAnnotationPath = annotationPath + str(i).zfill(4)

        imageData = cv2.imread(baseFilePath + ".png")

        annotations = np.genfromtxt(baseAnnotationPath + ".txt", delimiter=" ")

        results = []

        for a in annotations:
            if a[0] == 2.0:
                continue

            results.append(
                {
                    "xmin": a[1] - a[3] / 2.0,
                    "ymin": a[2] - a[4] / 2.0,
                    "xmax": a[1] + a[3] / 2.0,
                    "ymax": a[2] + a[4] / 2.0,
                    "label": labelSwitch.get(a[0]),
                }
            )

        imgHeight, imgWidth, _ = imageData.shape

        for res in results:
            xmin = int(res["xmin"] * imgWidth)
            ymin = int(res["ymin"] * imgHeight)
            xmax = int(res["xmax"] * imgWidth)
            ymax = int(res["ymax"] * imgHeight)
            label = res["label"]

            thick = int((imgHeight + imgWidth) // 600)

            cv2.rectangle(imageData, (xmin, ymin), (xmax, ymax), color, thick)

            y = ymin - 12
            if y < 0:
                y = ymax + 24

            cv2.putText(imageData, label, (xmin, y), 0, 1e-3 * imgHeight, color, thick)

        cv2.imwrite(baseOutputPath + ".png", imageData)
