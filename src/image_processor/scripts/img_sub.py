#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def img_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("Failed to convert image message: %s", e)
        return

    # 图像处理：检测红色物体
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # 使用两个范围覆盖鲜红色
    lower_red1 = np.array([0, 150, 80])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 80])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 使用形态学操作合并分割的区域
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算：膨胀 + 腐蚀

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # 找到面积最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 2000:  # 提高面积阈值
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 在方框左上角显示坐标
            text = f"({x}, {y})"
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            rospy.loginfo(f"Object detected at: ({x}, {y})")
    
    # 发布处理后的图像
    pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def img_subscriber():
    rospy.init_node('img_subscriber', anonymous=True)
    rospy.Subscriber('camera/image', Image, img_callback)
    global pub
    pub = rospy.Publisher('processed_image', Image, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        img_subscriber()
    except rospy.ROSInterruptException:
        pass
