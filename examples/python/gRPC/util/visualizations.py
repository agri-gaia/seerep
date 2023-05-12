# based on https://github.com/matterport/Mask_RCNN/blob/master/mrcnn/visualize.py
# Mask R-CNN

# The MIT License (MIT)

# Copyright (c) 2017 Matterport, Inc.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import colorsys
import random

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from matplotlib import patches


def random_colors(N, bright=True):
    """
    Generate random colors.
    To get visually distinct colors, generate them in HSV space then
    convert to RGB.
    """
    brightness = 1.0 if bright else 0.7
    hsv = [(i / N, 1, brightness) for i in range(N)]
    colors = list(map(lambda c: colorsys.hsv_to_rgb(*c), hsv))
    random.shuffle(colors)
    return colors


def display_instances(
    image,
    boxes,
    class_ids,
    class_names,
    scores=None,
    title="",
    figsize=(16, 16),
    ax=None,
    colors=None,
    captions=None,
    image_name="test.png",
    gli_image=False,
    rotate=False,
    offset=0,
    scale=1.0,
):
    """
    image: RGB image
    depth_image: respective Depth image for an RGB image
    boxes: [num_instance, (y1, x1, y2, x2, class_id)] in image coordinates.
    class_ids: [num_instances]
    class_names: list of class names of the dataset
    scores: (optional) confidence scores for each box
    title: (optional) Figure title
    figsize: (optional) the size of the image
    colors: (optional) An array or colors to use with each object
    captions: (optional) A list of strings to use as captions for each object
    """
    if rotate:
        image = np.rot90(np.rot90(image))
        for i in range(boxes.shape[0]):
            tempy1 = image.shape[0] - boxes[i][0]
            tempx1 = image.shape[1] - boxes[i][1]
            tempy2 = image.shape[0] - boxes[i][2]
            tempx2 = image.shape[1] - boxes[i][3]
            boxes[i][0] = tempy2
            boxes[i][1] = tempx2
            boxes[i][2] = tempy1
            boxes[i][3] = tempx1

    # Number of instances
    N = boxes.shape[0]
    if not N:
        print("\n*** No instances to display *** \n")
    else:
        assert boxes.shape[0] == class_ids.shape[0]

    # If no axis is passed, create one and automatically call show()
    auto_show = False
    if not ax:
        _, ax = plt.subplots(1, figsize=figsize)
        auto_show = True

    # Generate random colors
    colors = colors or random_colors(N)
    # plt.style.use('grayscale')
    # Show area outside image boundaries.
    height, width = image.shape[:2]
    ax.set_ylim(height + 10, -10)
    ax.set_xlim(-10, width + 10)
    ax.axis('off')
    ax.set_title(title)

    masked_image = image.astype(np.uint32).copy()
    for i in range(N):
        color = colors[i]

        # Bounding box
        if not np.any(boxes[i]):
            # Skip this instance. Has no bbox. Likely lost in image cropping.
            continue
        y1, x1, y2, x2 = boxes[i]
        p = patches.Rectangle(
            (x1, y1), x2 - x1, y2 - y1, linewidth=5, alpha=0.9, linestyle="dashed", edgecolor=color, facecolor='none'
        )
        ax.add_patch(p)

        # Label
        if not captions:
            class_id = class_ids[i]
            score = scores[i] if scores is not None else None
            label = class_names[class_id]
            caption = "{} {:.3f}".format(label, score) if score else label
        else:
            caption = captions[i]
        ax.text(x1 + 1, y1 + 13, caption, color='k', size=25, backgroundcolor=color, alpha=0.7)

    plot = ax.imshow(masked_image.astype(np.uint8))

    if gli_image:
        im_ratio = image.shape[0] / image.shape[1]
        cb = plt.colorbar(plot, fraction=0.046 * im_ratio, pad=0.04)

        def label_cbrt(x, pos):
            return "{:1.2f}".format((x / scale + offset) / 255.0 - 0.5)

        cb.formatter = ticker.FuncFormatter(label_cbrt)
        cb.update_ticks()

    plt.savefig(image_name, bbox_inches='tight')
    plt.close('all')
