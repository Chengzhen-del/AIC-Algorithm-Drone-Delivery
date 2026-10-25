#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import re
import subprocess
import sys

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def list_video_ids():
    ids = []
    for dev in sorted(glob.glob("/dev/video*")):
        m = re.search(r'/dev/video(\d+)$', dev)
        if m:
            ids.append(int(m.group(1)))
    return ids


def v4l2_list_formats_ext(camera_id):
    try:
        return subprocess.check_output(
            ["v4l2-ctl", "-d", "/dev/video%d" % camera_id, "--list-formats-ext"],
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=5,
        )
    except OSError:
        return False
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None


def parse_mjpeg_caps(text):
    caps = {}
    in_mjpeg = False
    current_wh = None
    for raw in text.splitlines():
        line = raw.strip()
        if re.match(r"\[\d+\]:", line):
            in_mjpeg = "MJPG" in line.upper()
            current_wh = None
            continue
        if not in_mjpeg:
            continue
        m = re.search(r"Size: Discrete (\d+)x(\d+)", line)
        if m:
            current_wh = (int(m.group(1)), int(m.group(2)))
            caps.setdefault(current_wh, [])
            continue
        m = re.search(r"\(([\d.]+) fps\)", line)
        if m and current_wh is not None:
            caps[current_wh].append(float(m.group(1)))
    return {wh: max(fps_list) for wh, fps_list in caps.items() if fps_list}


def log_camera_param_table(camera_id, mjpeg_caps, target_fps):
    if not mjpeg_caps:
        return
    lines = ["相机 MJPEG 参数表 (/dev/video%d), 目标帧率 %.3f fps:" % (camera_id, target_fps)]
    for (w, h) in sorted(mjpeg_caps.keys(), key=lambda wh: (-wh[0] * wh[1], -wh[0])):
        max_fps = mjpeg_caps[(w, h)]
        mark = "+" if max_fps + 1e-6 >= target_fps else " "
        lines.append("  [%s] %4d x %4d  max %.3f fps" % (mark, w, h, max_fps))
    rospy.loginfo("\n".join(lines))


def select_mjpeg_resolution(width, height, fps, mjpeg_caps):
    messages = []
    want = (int(width), int(height))

    if not mjpeg_caps:
        messages.append("警告: 未解析到 MJPEG 参数表，将直接使用请求分辨率 %dx%d" % want)
        return want[0], want[1], messages

    if want in mjpeg_caps and mjpeg_caps[want] + 1e-6 >= fps:
        return want[0], want[1], messages

    if want in mjpeg_caps:
        messages.append(
            "警告: 请求分辨率 %dx%d 在 MJPEG 下最高仅 %.3f fps，低于目标 %.3f fps"
            % (want[0], want[1], mjpeg_caps[want], fps)
        )
    else:
        messages.append(
            "警告: 相机 MJPEG 参数表中不包含请求分辨率 %dx%d" % (want[0], want[1])
        )

    candidates = [
        (wh[0] * wh[1], wh, mjpeg_caps[wh])
        for wh in mjpeg_caps
        if mjpeg_caps[wh] + 1e-6 >= fps
    ]
    if candidates:
        candidates.sort(key=lambda item: (-item[0], -item[2]))
        best_wh = candidates[0][1]
        messages.append(
            "已自动选择满足 %.3f fps 的最大 MJPEG 分辨率 %dx%d"
            % (fps, best_wh[0], best_wh[1])
        )
        return best_wh[0], best_wh[1], messages

    best_wh = max(mjpeg_caps.keys(), key=lambda wh: (wh[0] * wh[1], mjpeg_caps[wh]))
    messages.append(
        "警告: 没有任何 MJPEG 分辨率满足 %.3f fps，改用最大可用分辨率 %dx%d"
        % (fps, best_wh[0], best_wh[1])
    )
    return best_wh[0], best_wh[1], messages


def resolve_camera_by_v4l2(requested_id):
    ids = list_video_ids()
    if not ids:
        return None, None, True

    try_order = [requested_id] + [vid for vid in ids if vid != requested_id]
    for vid in try_order:
        raw = v4l2_list_formats_ext(vid)
        if raw is False:
            return None, None, False
        if raw is not None:
            if vid != requested_id:
                rospy.logwarn(
                    "请求的 /dev/video%d 无法通过 v4l2 检测，已切换到 /dev/video%d",
                    requested_id,
                    vid,
                )
            return vid, raw, True
    return None, None, True


def resolve_camera_by_opencv(requested_id):
    ids = list_video_ids()
    if not ids:
        return None

    try_order = [requested_id] + [vid for vid in ids if vid != requested_id]
    for vid in try_order:
        cap = cv2.VideoCapture(vid, cv2.CAP_V4L2)
        ok = cap.isOpened()
        cap.release()
        if ok:
            if vid != requested_id:
                rospy.logwarn(
                    "请求的 /dev/video%d 无法打开，已切换到 /dev/video%d",
                    requested_id,
                    vid,
                )
            return vid
    return None


def open_camera(camera_id, fps, width, height):
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if not cap.isOpened():
        cap.release()
        return None
    return cap


def main(argv):
    rospy.init_node("simple_camera_driver", argv=argv)
    camera_id = rospy.get_param("~camera_id", 0)
    frame_id = rospy.get_param("~frame_id", "mono_camera")
    rate_duration = rospy.get_param("~rate", 30)
    width = rospy.get_param("~width", 1920)
    height = rospy.get_param("~height", 1080)
    
    image_pub = rospy.Publisher("camera/image_raw", Image, queue_size=1)

    selected_id = None
    selected_caps_raw = None

    selected_id, selected_caps_raw, v4l2_available = resolve_camera_by_v4l2(camera_id)
    if not v4l2_available:
        rospy.loginfo("未检测到可用的 v4l2-ctl，将退回到 OpenCV 逐个尝试设备")
        selected_id = resolve_camera_by_opencv(camera_id)
        if selected_id is None:
            rospy.logerr("未找到可打开的摄像头设备")
            return
    elif selected_id is None:
        rospy.logwarn("未找到可通过 v4l2 检测的摄像头设备，将退回到 OpenCV 逐个尝试设备")
        selected_id = resolve_camera_by_opencv(camera_id)
        if selected_id is None:
            rospy.logerr("未找到可打开的摄像头设备")
            return

    if selected_caps_raw is not None:
        mjpeg_caps = parse_mjpeg_caps(selected_caps_raw)
        log_camera_param_table(selected_id, mjpeg_caps, rate_duration)
        width, height, messages = select_mjpeg_resolution(
            width, height, rate_duration, mjpeg_caps
        )
        for msg in messages:
            if msg.startswith("警告:"):
                rospy.logwarn("%s", msg)
            else:
                rospy.loginfo("%s", msg)

    capture = open_camera(selected_id, rate_duration, width, height)
    if capture is None:
        rospy.logerr("无法打开 /dev/video%d", selected_id)
        return

    actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = capture.get(cv2.CAP_PROP_FPS)
    if actual_width != int(width) or actual_height != int(height):
        rospy.logwarn(
            "驱动实际分辨率为 %dx%d，和设置的 %dx%d 不一致",
            actual_width,
            actual_height,
            int(width),
            int(height),
        )
    rospy.loginfo(
        "已打开 /dev/video%d, MJPEG %dx%d, 请求 fps=%.3f, 驱动返回 fps=%.3f",
        selected_id,
        actual_width,
        actual_height,
        rate_duration,
        actual_fps,
    )

    bridge = CvBridge()
    rate = rospy.Rate(rate_duration)
    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if ret:
            try:
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = frame_id
                image_pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(e)
        rate.sleep()


if __name__ == '__main__':
    main(sys.argv)

