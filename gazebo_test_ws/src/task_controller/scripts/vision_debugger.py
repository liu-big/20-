#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视觉探针节点：
这是一个最小化的测试脚本，用于验证ROS话题的订阅和发布是否通畅。
它不加载任何模型，只做最基本的消息收发。
"""
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class VisionDebuggerNode:
    def __init__(self):
        rospy.init_node('vision_debugger_node', anonymous=True)

        # 订阅正确的摄像头话题
        self.image_sub = rospy.Subscriber("/cam/Image", Image, self.image_callback, queue_size=1)

        # 发布到我们关心的目标话题
        self.detection_pub = rospy.Publisher("/vision/detections", Float32MultiArray, queue_size=1)

        rospy.loginfo("【探针】视觉探针节点已启动，正在等待来自 /cam/Image 的图像...")

        # 创建一个定时器，每2秒检查一次是否收到过图像
        self.received_image = False
        rospy.Timer(rospy.Duration(2), self.check_status)

    def image_callback(self, ros_image_msg):
        # 只要这个函数被调用，就说明我们成功收到了图像
        if not self.received_image:
            rospy.loginfo("【探针】成功收到第一张图像！ROS通信正常。")
            self.received_image = True

        # 创建一个固定的、虚拟的检测结果
        dummy_msg = Float32MultiArray()
        # 格式: [数量, class_id, score, x, y, w, h]
        dummy_msg.data = [1, 0.0, 0.99, 100.0, 100.0, 50.0, 50.0]

        # 发布这个虚拟结果
        self.detection_pub.publish(dummy_msg)

    def check_status(self, event):
        # 如果2秒后还未收到图像，就打印警告
        if not self.received_image:
            rospy.logwarn("【探针】警告：仍未收到任何图像。请检查 /cam/Image 话题是否有数据发布，以及话题名称是否正确。")

if __name__ == '__main__':
    try:
        node = VisionDebuggerNode()
        rospy.spin() # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("视觉探针节点已关闭。")
