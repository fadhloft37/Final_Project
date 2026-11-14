#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import vision_head as vh
 
color = sys.argv[1]      
bridge = CvBridge()
frame = None
roi = list(vh.ROI)
point = []
mode_roi = False

def callback(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')

def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and mode_roi:
        roi.append([x, y])

def main():
    global mode_roi
    rospy.init_node('calibrate', anonymous=True)
    rospy.Subscriber('/camera/image_raw', Image, callback, queue_size=1, buff_size=2**24)
    cv2.namedWindow('calibrate')
    cv2.setMouseCallback('calibrate', click)

    while not rospy.is_shutdown():
        if frame is None:
            cv2.waitKey(30)
            continue
        show = frame.copy()
        if len(roi) > 1:
            cv2.polylines(show, [np.array(roi, np.int32)], True, (255, 0, 0), 2)
        for u, v, x, y in point:
            cv2.circle(show, (int(u), int(v)), 4, (0, 255, 0), -1)
        cv2.putText(show, 'ROI' if mode_roi else 'TITIK: %d' % len(point), (10, 25), 1, 1.5, (255, 255, 255), 2)
        cv2.imshow('calibrate', show)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('r'):
            mode_roi = not mode_roi
        elif key == ord('u'):
            daftar = roi if mode_roi else point
            if daftar:
                daftar.pop()
        elif key == ord('a'):
            result = vh.find_object(frame, color, roi)
            if result is None:
                print("Objek tidak terdeteksi")
                continue
            try:
                x, y = [float(t) for t in input('x y (meter, pisah spasi) > ').replace(',', '.').split()]
            except ValueError:
                print('Format salah')
                continue
            point.append((result[0], result[1], x, y))
        elif key == ord('h'):
            train, test = point[:-3], point[-3:]
            if len(train) < 4:
                print('Titik kurang (minimal 8 latih + 3 uji)')
                continue
            H = cv2.findHomography(np.float32([t[:2] for t in train]),
                                   np.float32([t[2:] for t in train]), 0)[0]
            H = H / H[2, 2]
            for name, group in(('LATIH', train), ('UJI', test)):
                galat = []
                for u, v, x, y in group:
                    px, py = vh.distance(u, v, H)
                    galat.append(1000 * np.hypot(px - x, py - y))
                print(name, 'error (mm):', np.round(galat, 1))
            print('UJI:', 'LULUS' if max(galat) < 5 else 'BELUM LULUS (target < 5 mm)')
            print('--- tempel 3 baris ini ke vision_utils.py ---')
            print('UKURAN_GAMBAR =', (frame.shape[1], frame.shape[0]))
            print('ROI =', roi)
            print('H =', H.tolist())
    cv2.destroyAllWindow()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

