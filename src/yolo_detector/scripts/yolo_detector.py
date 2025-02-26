#!/usr/bin/env python
import rospy
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String  # 用于发布检测结果

def yolo_detector():
    rospy.init_node("yolo_detector")
    pub = rospy.Publisher("detected_object", String, queue_size=10)
    cap = cv2.VideoCapture(0)  # 使用摄像头
    model = YOLO('yolov8n.pt')  # 加载 YOLOv8 模型
    bridge = CvBridge()

    while not rospy.is_shutdown():
        success, frame = cap.read()
        if not success:
            rospy.logwarn("Failed to read frame from camera")
            break

        # 运行 YOLO 检测
        results = model(frame)  # 返回的结果是一个列表

        # 遍历检测结果
        for result in results:
            boxes = result.boxes.cpu().numpy()  # 获取检测框并转移到 CPU
            for box in boxes:
                xmin, ymin, xmax, ymax = box.xyxy[0]  # 获取检测框的坐标
                confidence = box.conf[0]  # 获取置信度
                class_id = int(box.cls[0])  # 获取类别ID
                class_name = model.names[class_id]  # 获取类别名称

                if confidence > 0.5:  # 置信度阈值
                    rospy.loginfo(f"Detected: {class_name} at ({xmin}, {ymin}, {xmax}, {ymax})")

                    # 发布检测到的物体类别名称和坐标
                    msg = String()
                    msg.data = f"{class_name}: ({xmin}, {ymin}, {xmax}, {ymax})"
                    pub.publish(msg)

        # 显示检测结果（可选）
        annotated_frame = results[0].plot()  # 绘制检测框
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        yolo_detector()
    except rospy.ROSInterruptException:
        pass