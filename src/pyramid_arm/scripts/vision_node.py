#!/usr/bin/env python3
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import vision

bridge = CvBridge()
pub = None
target_color = None
H = None
hsv_table = None
show = True
calib = False

# SET TARGETED COLOR
def color_callback(msg):
    global target_color
    target_color = msg.data
    rospy.loginfo("warna target: %s", target_color)

# CONVERT IMG TO CV2 FORMAT
def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    except CvBridgeError as e:
        rospy.logerr("cv_bridge: %s", e)
        return
    process(frame)

# MAIN PROCESS
def process(frame):
    display = frame
    mask = None

    # DETECT
    if target_color in hsv_table:
        found, x, y, info = vision.detect(frame, target_color, H, hsv_table)
        mask = info["mask"]
        if found:
            pub.publish(Point(x=x, y=y))
            display = vision.draw(frame, info, x, y)
        else:
            rospy.logwarn_throttle(2, "warna %s tidak terdeteksi", target_color)

    # DISPLAY
    if calib:
        display = vision.draw_calibration(display)

    if not show:
        return

    if mask is None:
        mask = np.zeros(frame.shape[:2], np.uint8)
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    cv2.putText(display, "target: %s" % target_color, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    gabung = cv2.resize(np.hstack([display, mask_bgr]), None, fx=0.6, fy=0.6)
    cv2.imshow("vision", gabung)
    cv2.waitKey(1)


def main():
    global H, hsv_table, show, pub, calib

    rospy.init_node("vision_node", anonymous=True)
    mode = rospy.get_param("~mode", "sim")
    show = rospy.get_param("~show", True)
    calib = rospy.get_param("~calib", True)

    hsv_table = vision.HSV_SIM if mode == "sim" else vision.HSV_HW
    H = vision.buat_H()

    pub = rospy.Publisher("/block_pos", Point, queue_size=1)
    rospy.Subscriber("/target_color", String, color_callback)
    source = rospy.get_param("~source", "topic")

    # ACTION ACCORDING TO MODE
    if source == "topic":
        rospy.Subscriber("/camera/image_raw", Image, image_callback, queue_size=1)
        rospy.loginfo("vision_node ready, mode = %s, source = topic", mode)
        rospy.spin()
        return  # penting: jangan lanjut ke blok kamera

    cam_id = rospy.get_param("~camera_id", "http://10.161.126.195:8080/video")
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        rospy.logerr("Kamera %s tidak bisa dibuka", cam_id)
        return

    w = rospy.get_param("~width", 0)
    h = rospy.get_param("~height", 0)
    if w > 0 and h > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

    rospy.loginfo("vision_node ready, mode = %s, source = camera %s", mode, cam_id)
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn_throttle(2, "Gagal membaca frame dari kamera")
            else:
                process(frame)
            rate.sleep()
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()