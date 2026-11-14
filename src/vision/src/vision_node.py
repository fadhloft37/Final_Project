#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import vision_head as vh
# from fp_msgs.msg import BlockPose

bridge = CvBridge()
target = None

def color_callback(msg):
    global target
    if msg.data in vh.HSV:
        target = msg.data
    else:
        rospy.logwarn('Warna tidak dikenal')

def img_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    if(frame.shape[1], frame.shape[0]) != vh.IMG_SZ:
        rospy.logwarn_throttle(5, 'Resolusi beda dari saat kalibrasi!')
    debug = frame.copy()
    if len(vh.ROI) > 2:
        cv2.polylines(debug, [np.array(vh.ROI, np.int32)], True, (255, 0, 0), 2)

    if target is not None:
        found, x, y = 0, 0.0, 0.0
        result = vh.find_object(frame, target)
        if result is not None:
            u, v, contour = result
            cv2.drawContours(debug, [contour], -1, (0, 255, 0), 2)
            cv2.circle(debug, (int(u), int(v)), 5, (0, 0, 255), -1)
            if vh.H is not None:
                x, y = vh.distance(u, v)
                found = 1
            else:
                rospy.logwarn_throttle(5, 'Belum kalibrasi (H masih None)')
        pub_pose.publish(String('%s %d %.4f %.4f' % (target, found, x, y)))
        # p = BlockPose()
        # p.header.stamp = msg.header.stamp
        # p.header.frame_id = 'base'
        # p.color = target
        # p.found = bool(found)
        # p.x = x
        # p.y = y
        # pub_pose.publish(p)
    pub_debug.publish(bridge.cv2_to_imgmsg(debug, 'bgr8'))

def main():
    global pub_pose, pub_debug
    rospy.init_node('vision')
    pub_pose = rospy.Publisher('/vision/block_pose_text', String, queue_size=1)
    # pub_pose = rospy.Publisher('/vision/block_pose', BlockPose, queue_size=1)          
    pub_debug = rospy.Publisher('/vision/debug_image', Image, queue_size=1)
    rospy.Subscriber('/vision/target_color', String, color_callback)
    rospy.Subscriber('/camera/image_raw', Image, img_callback, queue_size=1, buff_size=2**24)
    rospy.loginfo('Vision siap')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
  