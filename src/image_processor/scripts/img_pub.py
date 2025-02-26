#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def img_publisher():
    rospy.init_node('img_publisher', anonymous=True)
    pub = rospy.Publisher('camera/image', Image, queue_size=10)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)  # 使用摄像头

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            rospy.loginfo("Publishing image")
            pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        else:
            rospy.logwarn("Failed to capture image")
        rospy.sleep(0.1)  # 控制发布频率

if __name__ == '__main__':
    try:
        img_publisher()
    except rospy.ROSInterruptException:
        pass
