#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

if sys.prefix != sys.base_prefix:
    _venv_site = os.path.join(
        sys.prefix, 'lib',
        'python{}.{}'.format(sys.version_info.major, sys.version_info.minor),
        'site-packages')
    if os.path.isdir(_venv_site):
        if _venv_site in sys.path:
            sys.path.remove(_venv_site)
        sys.path.insert(0, _venv_site)

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path
from tutorial_vision_feng.msg import StringStamped
import torch
import torch.nn.functional as F
from std_srvs.srv import Empty, EmptyResponse


ROOT = ''
if __name__ == '__main__':
    rospy.init_node('yolov5_predict_node', sys.argv)
    ROOT = rospy.get_param('~yolov5_root')
else:
    FILE = Path(__file__).resolve()
    ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))


from models.common import DetectMultiBackend
from utils.augmentations import classify_transforms
from utils.general import LOGGER, check_requirements, colorstr, increment_path, print_args, set_logging
from utils.torch_utils import select_device, smart_inference_mode, time_sync


def load_model(weights, device='', imgsz=64, half=False, dnn=False):
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, fp16=half)
    model.warmup(imgsz=(1, 3, imgsz, imgsz))
    transforms = classify_transforms(imgsz)
    rospy.logwarn("YOLOv5 model loaded on %s" % device)
    return model, transforms, device


@smart_inference_mode()
def detect(model, transforms, device, source, confidence=0.85):
    im = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
    im = transforms(im).unsqueeze(0).to(device)
    im = im.half() if model.fp16 else im.float()

    results = model(im)

    p = F.softmax(results, dim=1)
    top5_indices = p.argsort(1, descending=True)[:, :5].squeeze()
    top5_confidences = p[0, top5_indices]

    top5_classes = [model.names[idx] for idx in top5_indices]
    message = ""
    for cls, conf in zip(top5_classes, top5_confidences):
        message = message + "{}:{}, ".format(cls, conf)
    #rospy.loginfo(message)

    return model.names[top5_indices[0]] if torch.max(p) > confidence else ''


def cut_circumscribed_square(origin_image):
    # cv2.imshow('origin', origin_image)
    image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image, (21, 21), 0)
    height, width = image.shape
    _, image = cv2.threshold(src=image, thresh=100, maxval=255,
                             type=cv2.THRESH_BINARY)
    # cv2.imshow('binary', image)
    result = cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, 1, int(max(width, height)), param1=1350,
                              param2=20, minRadius=50,
                              maxRadius=int(max(width, height)/2))
    if result is not None:
        result = np.uint16(np.around(result).reshape(-1, 3))
        x, y, r = result[0]
        r = r*0.66
        min_x = int(max(0, x-r))
        max_x = int(min(width-1, x+r))
        min_y = int(max(0, y-r))
        max_y = int(min(height-1, y+r))
        if min_x >= max_x or min_y >= max_y:
            return None
        cut = origin_image[min_y:max_y, min_x:max_x] 
        return cut
    return None


def image_cb(msg, bridge, model, transforms, device, pub, confidence):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    input = cut_circumscribed_square(cv_image)
    if input is not None:
        # cv2.imshow("detect input", input)
        # cv2.waitKey(1)
        name = detect(model, transforms, device, input, confidence=confidence)
        if name != '':
            result = StringStamped()
            result.header = msg.header
            result.data.append(name)
            pub.publish(result)


enable_yolo_detect = False

def toggleSubscribe(req):
    global enable_yolo_detect
    if enable_yolo_detect:
        rospy.loginfo("disable yolo_detect")
        enable_yolo_detect = False
    else:
        rospy.loginfo("enable yolo_detect")
        enable_yolo_detect = True
    return EmptyResponse()


def main():
    set_logging(verbose=False)
    weights = rospy.get_param('~weights_path')
    device = rospy.get_param('~device', '')
    confidence = rospy.get_param('~confidence', 0.85)
    model, transforms, device = load_model(weights, device=device)
    bridge = CvBridge()
    str_pub = rospy.Publisher('yolo_detect', StringStamped, queue_size=1)
    rospy.Service('toggle_yolo_detect', Empty, toggleSubscribe)
    rospy.Subscriber('camera/image_raw', Image,
        lambda msg: image_cb(msg, bridge, model, transforms, device, str_pub, confidence)
    )
    rospy.spin()


if __name__ == '__main__':
    main()
