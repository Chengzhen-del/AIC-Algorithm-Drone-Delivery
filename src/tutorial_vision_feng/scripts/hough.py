import cv2
import numpy as np
from ultralytics import YOLO


def hough(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image, (21, 21), 0)
    height, width = image.shape

    # _, image1 = cv2.threshold(src=image, thresh=180, maxval=255,
    #                           type=cv2.THRESH_BINARY_INV)
    # cv2.imshow('hough input 1', image1)
    # result1 = cv2.HoughCircles(image1, cv2.HOUGH_GRADIENT, 1, int(max(width, height)), param1=135,
    #                            param2=20, minRadius=50,
    #                            maxRadius=int(max(width, height)/2))

    _, image2 = cv2.threshold(src=image, thresh=100, maxval=255,
                              type=cv2.THRESH_BINARY)
    cv2.imshow('hough input 2', image2)
    result2 = cv2.HoughCircles(image2, cv2.HOUGH_GRADIENT, 1, int(max(width, height)), param1=1350,
                               param2=30, minRadius=50,
                               maxRadius=int(max(width, height)/2))
    result = None
    if result2 is not None:
        result2 = np.uint16(np.around(result2).reshape(-1, 3))
        result = result2[0]
    return result
    # if result1 is not None and result2 is not None:
    #     result1 = np.uint16(np.around(result1.reshape(-1, 3)))
    #     result2 = np.uint16(np.around(result2).reshape(-1, 3))
    #     for r1 in result1:
    #         for r2 in result2:
    #             dis = (r1[0] - r2[0])**2+(r1[1]-r2[1])**2
    #             print(dis)
    #             if dis < 10:
    #                 result = r1

    # return result


model = YOLO('./best.pt')

# --------------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
waitTime = 1
while True:
    ret, frame = cap.read()
    if not ret:
        break
# --------------

# import glob
# img_path = glob.glob("./sim/cifar100/test/*.png")
# waitTime = -1
# for path in img_path:
#     frame = cv2.imread(path)
#     print(path)

# --------------
    circle = hough(frame)
    # cutoff = np.zeros(frame.shape, np.uint8)
    if circle is not None:
        x, y, r = circle
        # r = int(r*0.66)
        # for (x, y, r) in circles[0, :]:
        height, width, _ = frame.shape
        minX = max(0, x-r)
        maxX = min(width-1, x+r)
        minY = max(0, y-r)
        maxY = min(height-1, y+r)
        # cutoff[y-r:y+r, x-r:x+r] = frame[y-r:y+r, x-r:x+r]
        # yolo_input = frame[y-r:y+r, x-r:x+r]
        # if yolo_input.shape[0] > 1 and yolo_input.shape[1] > 1:
        #     results = model.predict(yolo_input, imgsz=64, device='mps',
        #                             show=True, save=False, retina_masks=True,
        #                             conf=0.8, half=True)
        #     for result in results:
        #         print(cifar100_labels[result.probs.top1]+": " +
        #               str(result.probs.top1conf.item()))
        # 绘制外圆
        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
        # 绘制圆心
        cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)

    # 显示结果
    # cv2.imshow('cutoff', cutoff)
    cv2.imshow('Frame', frame)

    key = cv2.waitKey(waitTime)
    if key & 0xFF == ord('q'):
        break
