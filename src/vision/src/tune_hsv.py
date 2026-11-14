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

def callback(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')

def main():
    rospy.init_node('tune_hsv', anonymous=True)
    rospy.Subscriber('/camera/image_raw', Image, callback, queue_size=1, buff_size=2**24)

    cv2.namedWindow('tune')
    name = ['Hmin', 'Smin', 'Vmin', 'Hmax', 'Smax', 'Vmax']
    initial = vh.HSV[color][0] + vh.HSV[color][1]
    for n, v in zip(name, initial):
        cv2.createTrackbar(n, 'tune', v, 179 if n[0] == 'H' else 255, lambda x: None)

    while not rospy.is_shutdown():
        if frame is None:
            cv2.waitKey(30)
            continue
        p = [cv2.getTrackbarPos(n, 'tune') for n in name]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(p[:3]), np.array(p[3:]))
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('tune', np.concatenate([frame, result], axis=1))
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            print("     '%s': (%s %s)," % (color, p[:3], p[3:]))
    cv2.destroyAllWindow()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
