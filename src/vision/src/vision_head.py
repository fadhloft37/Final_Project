import cv2
import numpy as np

IMG_SZ = (640, 480)
AREA_MIN = 150
HSV = {
    'orange': ([5, 140, 100], [20, 255, 255]),
    'green': ([40, 70, 50], [85, 255, 255]),
    'beige': ([10, 20, 140], [35, 110, 255])
}
ROI = []
H = None

def find_object(frame, color, roi=None):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low, high = HSV[color]
    mask = cv2.inRange(hsv, np.array(low), np.array(high))

    roi = ROI if roi is None else roi
    area = np.zeros(mask.shape, np.uint8)
    if len(roi) > 2:
        cv2.fillPoly(area, [np.array(roi, np.int32)], 255)
    else:
        area[:] = 255
    mask = cv2.bitwise_and(mask, area)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contour, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = [k for k in contour if cv2.contourArea(k) > AREA_MIN]
    if not contour:
        return None
    k = max(contour, key=cv2.contourArea)
    m = cv2.moments(k)
    return m['m10'] / m['m00'], m['m01'] / m['m00'], k

def distance(u, v, matrix=None):
    matrix = H if matrix is None else matrix
    p = cv2.perspectiveTransform(np.float32([[[u, v]]]), np.array(matrix, np.float64))
    return float(p[0][0][0]), float(p[0][0][1])
