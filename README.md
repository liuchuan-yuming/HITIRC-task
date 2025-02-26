# HITIRC-task
## 任务概述
本报告记录了我在Aelos&Roban任务中的完成情况，主要包括“1.0学习概览”、“1.1基本任务”和“1.2进阶任务”。以下是任务的具体内容及实现情况。

## 1.0 学习概览
### ROS学习
根据b站ros课程，同步学习文件在src中：
- learning_topic
- learning_service
- learning_parameter
- learning_tf
- learning_launch
参考学习笔记 [ROS学习笔记](https://blog.csdn.net/takedachia/category_11584500.html)

## 1.1 基本任务：使用OpenCV识别红色物体
### 任务目标
通过OpenCV实现对红色物体的识别。

### 实现方法

- 使用OpenCV库对图像进行处理，通过颜色空间转换（从RGB到HSV）和颜色阈值筛选，提取红色物体的轮廓。
- 编写launch文件简化启动流程，方便快速运行程序。
    
### 提交材料

- 模块名称：image_processor
- 功能描述：使用OpenCV识别红色物体。
- 启动方式：通过launch文件启动，简化了程序运行流程。
## 1.2 进阶任务：使用YOLOv8进行物体识别
### 任务目标
通过YOLOv8实现对图像或视频中的物体进行识别。

### 实现方法

- 使用预训练的YOLOv8模型进行物体检测。

### 提交材料

- 模块名称：yolo_detecter
- 功能描述：使用YOLOv8模型进行物体检测。
