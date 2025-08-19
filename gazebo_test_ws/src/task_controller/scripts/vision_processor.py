#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视觉处理节点 (v4 - 可配置版)
- 通过ROS参数服务器动态读取要监听的摄像头话题。
- 包含了完整的ONNX模型处理逻辑。
"""

import rospy
import cv2
import numpy as np
import onnxruntime
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import os

# --- 这部分代码和之前保持一致 ---
# [此处省略所有模型处理函数(letterbox, dfl, post_process等)，请保留你文件中已有的这些函数]
ONNX_MODEL_PATH = "/home/liu/ucar_ws/gazebo_test_ws/src/task_controller/scripts/best.onnx"
MODEL_INPUT_SHAPE = (640, 640)
OBJ_THRESH, NMS_THRESH = 0.4, 0.25
CLASSES = ("milk","cake","coke","apple","banana","watermelon","pepper","tomato","potato","red_light","green_light")

def letterbox(im, new_shape=(640, 640), color=(114, 114, 114)):
    shape = im.shape[:2];r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2; dh /= 2
    if shape[::-1] != new_unpad: im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1)); left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)
def dfl(position):
    n, c, h, w = 1, position.shape[1], position.shape[2], position.shape[3]; position = position.reshape(n, 4, c//4, h, w)
    p_num = 4; mc = c // p_num; y = position.reshape(n, p_num, mc, h, w)
    e_y = np.exp(y - np.max(y, axis=2, keepdims=True)); y = e_y / np.sum(e_y, axis=2, keepdims=True)
    acc_metrix = np.arange(mc).reshape(1, 1, mc, 1, 1); y = (y * acc_metrix).sum(2)
    return y
def box_process(position):
    grid_h, grid_w = position.shape[2:4]; col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    grid = np.stack((col, row), axis=0).reshape(1, 2, grid_h, grid_w) + 0.5
    stride = np.array([MODEL_INPUT_SHAPE[1] / grid_w, MODEL_INPUT_SHAPE[0] / grid_h]).reshape(1, 2, 1, 1)
    box_dist = dfl(position); lt = grid - box_dist[:, 0:2, :, :]; rb = grid + box_dist[:, 2:4, :, :]
    lt_stride = lt * stride; rb_stride = rb * stride
    xyxy = np.concatenate((lt_stride, rb_stride), axis=1)
    return xyxy.reshape(4, -1).T
def post_process(outputs, ratio, dwdh):
    output_pairs = []; i=0
    while i < len(outputs):
        if outputs[i].shape[1] == 64 and i+1 < len(outputs) and outputs[i+1].shape[1] == len(CLASSES):
            output_pairs.append((outputs[i], outputs[i+1])); i += 2
        else: i+=1
    all_boxes, all_scores, all_class_ids = [], [], []
    for box_output, cls_output in output_pairs:
        all_boxes.append(box_process(box_output))
        cls_scores = cls_output.reshape(cls_output.shape[1], -1).T; all_scores.append(np.max(cls_scores, axis=1)); all_class_ids.append(np.argmax(cls_scores, axis=1))
    boxes_final = np.concatenate(all_boxes); scores_final = np.concatenate(all_scores); class_ids_final = np.concatenate(all_class_ids)
    mask = scores_final > OBJ_THRESH; boxes_unpad = boxes_final[mask]
    boxes_unpad[:, 0] = (boxes_unpad[:, 0] - dwdh[0]) / ratio; boxes_unpad[:, 1] = (boxes_unpad[:, 1] - dwdh[1]) / ratio
    boxes_unpad[:, 2] = (boxes_unpad[:, 2] - dwdh[0]) / ratio; boxes_unpad[:, 3] = (boxes_unpad[:, 3] - dwdh[1]) / ratio
    widths = boxes_unpad[:, 2] - boxes_unpad[:, 0]; heights = boxes_unpad[:, 3] - boxes_unpad[:, 1]
    nms_boxes = np.column_stack((boxes_unpad[:, 0], boxes_unpad[:, 1], widths, heights)).tolist(); nms_scores = scores_final[mask].tolist()
    indices = cv2.dnn.NMSBoxes(nms_boxes, nms_scores, score_threshold=OBJ_THRESH, nms_threshold=NMS_THRESH)
    if len(indices) == 0: return [], [], []
    final_boxes_list = [nms_boxes[i] for i in indices]; final_scores_list = [nms_scores[i] for i in indices]; final_classes_list = [class_ids_final[mask][i] for i in indices]
    return final_boxes_list, final_scores_list, final_classes_list
def draw_detections(image, boxes, scores, classes):
    for box, score, cl in zip(boxes, scores, classes):
        left, top, width, height = map(int, box); cv2.rectangle(image, (left, top), (left + width, top + height), (0, 255, 0), 2)
        label = f'{CLASSES[cl]}: {score:.2f}'; cv2.putText(image, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return image
    
class VisionProcessorNode:
    def __init__(self):
        rospy.init_node('vision_processor_node', anonymous=True)

        # 【核心修正】从参数服务器读取话题名称
        # 如果launch文件中没有设置，就使用默认值 "/cam/Image"
        camera_topic = rospy.get_param("~camera_topic", "/topic_not_set_in_launch_file")
        rospy.loginfo(f"【配置】将从话题 '{camera_topic}' 订阅图像...")

        # ... [加载模型和创建发布器的代码和之前一样] ...
        if not os.path.exists(ONNX_MODEL_PATH):
            rospy.logfatal(f"错误: ONNX模型文件未找到！路径: {ONNX_MODEL_PATH}")
            return
        rospy.loginfo("正在加载ONNX模型...")
        self.session = onnxruntime.InferenceSession(ONNX_MODEL_PATH, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name; self.output_names = [o.name for o in self.session.get_outputs()]
        rospy.loginfo("模型加载成功。")
        self.bridge = CvBridge()
        self.debug_image_pub = rospy.Publisher("/vision/debug_image", Image, queue_size=1)
        self.detection_pub = rospy.Publisher("/vision/detections", Float32MultiArray, queue_size=1)
        
        # 使用读取到的话题名称来创建订阅者
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("视觉处理节点已启动。")

    def image_callback(self, ros_image_msg):
        # ... [回调函数的内部逻辑和之前一样，无需修改] ...
        try:
            cv_frame = self.bridge.imgmsg_to_cv2(ros_image_msg, "bgr8")
            img_resized, ratio, dwdh = letterbox(cv_frame)
            img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
            img_tensor = np.transpose(img_rgb, (2, 0, 1)).astype(np.float32) / 255.0
            input_tensor = np.expand_dims(img_tensor, axis=0)
            outputs = self.session.run(self.output_names, {self.input_name: input_tensor})
            boxes, scores, classes = post_process(outputs, ratio, dwdh)
            detection_msg = Float32MultiArray()
            detection_data = [len(boxes)]
            for box, score, cl in zip(boxes, scores, classes):
                detection_data.extend([float(cl), float(score)] + [float(b) for b in box])
            detection_msg.data = detection_data
            self.detection_pub.publish(detection_msg)
            debug_frame = draw_detections(cv_frame, boxes, scores, classes)
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))
        except Exception as e:
            rospy.logerr(f"回调函数中发生错误: {e}")

if __name__ == '__main__':
    try:
        node = VisionProcessorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass