import cv2
import math
import numpy as np

# HSV RANGE IN SIM
HSV_SIM = {
    "orange": ((5, 120, 120),  (20, 255, 255)),
    "black":  ((0, 0, 0),      (179, 255, 60)),
    "beige":  ((15, 20, 150),  (30, 90, 255)),
}

#HSV RANGE IN HARDWARE
HSV_HW = {
    "orange": ((0, 90, 130),  (25, 255, 255)),
    "black":  ((0, 0, 0), (115, 45, 75)),
    "beige":  ((0, 0, 175),    (35, 80, 255)),
}

KERNEL =  np.ones((5, 5), np.uint8)
MIN_AREA = 300
IMG_W = 640                  
IMG_H = 480                  
LEBAR_BALOK_PX = 50      
LEBAR_BALOK_M = 0.05    
IGNORE = [(500,300,640,480),]     

# MASKING WANTED COLOR
def masking(frame, color, hsv_range=HSV_HW):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    low, high = hsv_range[color]

    mask = cv2.inRange(hsv, np.array(low), np.array(high))

    for (x1, y1, x2, y2) in IGNORE:
        mask[y1:y2, x1:x2] = 0
    return mask

# HOMOGRAPHY
def buat_H():
    s = LEBAR_BALOK_M / LEBAR_BALOK_PX 
    bx, by = IMG_W / 2, IMG_H
    return np.array([[ 0, -s, s * by],
                     [-s,  0, s * bx],
                     [ 0,  0, 1     ]], dtype=np.float64)

# CONVERT PIXEL TO METER
def px2m(H, px, py):
    point = np.array([[[px, py]]], dtype=np.float32)
    result = cv2.perspectiveTransform(point, H)
    return float(result[0][0][0]), float(result[0][0][1])

# CHOOSING THE TARGET
def detect(frame, color, H, hsv_range=HSV_HW, min_area=MIN_AREA):
    mask = masking(frame, color, hsv_range)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ =cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for c in contours:
        if cv2.contourArea(c) < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        cx, cy = x+w // 2, y+h // 2
        mx, my = px2m(H, cx, cy)
        distance = math.hypot(mx, my)
        if best is None or distance < best["distance"]:
            best = dict(cx=cx, cy=cy, bbox=(x, y, w, h), mx=mx, my=my, distance=distance)

    if best is None:
        return False, 0.0, 0.0, {"mask": mask}

    best["mask"] = mask
    return True, best["mx"], best["my"], best

# DRAW BOUNDING BOX AND IGNORE AREA
def draw(frame, info, x_m=None, y_m=None):
    out = frame.copy()
    for (x1, y1, x2, y2) in IGNORE:
        cv2.rectangle(out, (x1, y1), (x2, y2), (255, 0, 0), 1)
    x, y, w, h = info["bbox"]
    cv2.rectangle(out, (x,y), (x+w, y+h), (0,255,0), 2)
    cv2.circle(out, (info["cx"], info["cy"]), 5, (0,0,255), -1)
    if x_m is not None:
        teks = "x=%.3f y=%.3f m" % (x_m, y_m)
        cv2.putText(out, teks, (x, max(y - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out

# DRAW TOOLS IN CAMERA TO HELP CALIBRATE
def draw_calibration(frame):
    out = frame.copy()
    bx = int(round(IMG_W / 2))         
    by = IMG_H - 1                       
    s  = LEBAR_BALOK_M / LEBAR_BALOK_PX   
    step = int(round(LEBAR_BALOK_PX))     

    # BLOCK REFERENCE
    cv2.rectangle(out, (bx - step, by - step), (bx, by), (255, 0, 255), 1)
    cv2.putText(out, "5x5 cm", (bx - step + 2, by - step - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    # ORIGIN
    cv2.drawMarker(out, (bx, by), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(out, "O (0,0)", (bx + 10, by - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # ARROW X
    cv2.arrowedLine(out, (bx, by), (bx, by - 70),
                    (0, 200, 255), 2, tipLength=0.15)
    cv2.putText(out, "+x", (bx - 20, by - 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    # ARROW Y
    cv2.arrowedLine(out, (bx, by), (bx - 90, by),
                    (0, 200, 255), 2, tipLength=0.12)
    cv2.putText(out, "+y", (bx - 105, by - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    # SCALE
    yb = by - step - 10
    cv2.line(out, (bx - step, yb), (bx, yb), (0, 255, 0), 2)
    cv2.putText(out, "%d cm" % round(LEBAR_BALOK_M * 100),
                (bx - step + 2, yb - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    txt = "s=%.3f mm/px  %d px = %d cm  origin=(%d,%d)" % (
        s * 1000, LEBAR_BALOK_PX, round(LEBAR_BALOK_M * 100), bx, by)
    cv2.putText(out, txt, (5, IMG_H - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    return out