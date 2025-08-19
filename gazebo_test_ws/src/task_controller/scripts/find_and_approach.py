#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
最终版任务控制器（v18.3 - 闪电返航版）：
- 根据用户要求，返航逻辑升级为“机会主义导航”，进入指定的安全区即可，不再进行不必要的姿态调整，以节省时间。
- 引入“自主解困机动”：当导航被卡住时，机器人会主动执行微操（后退、转身）来脱困。
- 导航逻辑为“机会主义”+“自主解困”的终极形态，追求极致的流畅、快速与适应性。
"""

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Float32MultiArray, Int32, String
import math
import json
import websocket
import threading
import random

class TaskController:
    # ==================== 可调参数区 ====================
    ROOM_SEARCH_AREAS = {
        'A': {'x': (0.3, 0.7), 'y': (1.0, 1.4), 'yaw': 1.57},
        'B': {'x': (1.8, 2.2), 'y': (1.0, 1.4), 'yaw': 1.57},
        'C': {'x': (3.7, 4.1), 'y': (1.0, 1.4), 'yaw': 1.57},
    }
    # [核心修改] 将返航目标从精确点改为一个“安全区”
    HOME_SAFE_AREA = {'x': (-0.1, 0.3), 'y': (-0.1, 0.3), 'yaw': 1.57}

    NAVIGATION_TIMEOUT = 45.0
    MONITOR_RATE = 5.0
    TOTAL_MISSION_TIMEOUT = 800.0
    SEARCH_ROTATION_SPEED = 1.2
    PEEK_ANGLE_RANGE = math.pi / 3

    # 自主解困参数
    STUCK_TIMEOUT = 3.0
    RECOVERY_BACKWARD_DIST = -0.15
    RECOVERY_TURN_ANGLE = 0.35

    ROSBRIDGE_URL = "ws://192.168.31.210:9090"
    
    ITEM_TO_CATEGORY_MAP = {
        0: 0, 1: 0, 2: 0, 
        3: 1, 4: 1, 5: 1,
        6: 2, 7: 2, 8: 2, 
        9: 3, 10: 3
    }

    ITEM_ID_TO_NAME_MAP = {
        0: "milk", 1: "cake", 2: "coke",
        3: "apple", 4: "banana", 5: "watermelon",
        6: "pepper", 7: "tomato", 8: "potato"
    }
    # =======================================================

    def __init__(self):
        rospy.init_node('task_controller_node', anonymous=False)
        
        while rospy.Time.now() == rospy.Time(0) and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "正在等待ROS时间同步...")
            rospy.sleep(0.1)
        
        self.mission_start_time = rospy.Time.now()
        self.simulation_timeout = rospy.Duration(self.TOTAL_MISSION_TIMEOUT)
        
        self.tf_listener = tf.TransformListener()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("正在等待move_base服务...")
        self.move_base_client.wait_for_server()
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.target_category_id = -1
        self.found_category_id = -1
        self.found_target_location = None
        self.current_detections = []

        self.found_item_id = -1
        self.found_item_name = ""
        self.vision_sub = rospy.Subscriber("/vision/detections", Float32MultiArray, self.vision_callback)
        
        self.ws_app = None
        self.ws_thread = None
        self.is_ws_connected = False
        self.setup_websocket()

        rospy.loginfo("任务控制器 v18.3 (闪电返航版) 已启动。")

    def setup_websocket(self):
        self.ws_app = websocket.WebSocketApp(self.ROSBRIDGE_URL, on_open=self.on_ws_open, on_message=self.on_ws_message, on_error=self.on_ws_error, on_close=self.on_ws_close)
        self.ws_thread = threading.Thread(target=self.ws_app.run_forever)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        rospy.loginfo(f"正在尝试连接到 rosbridge: {self.ROSBRIDGE_URL}")

    def on_ws_open(self, ws):
        rospy.loginfo("rosbridge 连接成功！正在订阅任务话题...")
        self.is_ws_connected = True
        subscribe_message = {"op": "subscribe", "topic": "/mission_target", "type": "std_msgs/Int32"}
        ws.send(json.dumps(subscribe_message))
        rospy.loginfo("已通过 rosbridge 订阅 /mission_target 话题。")

    def on_ws_message(self, ws, message):
        data = json.loads(message)
        if data.get("op") == "publish" and data.get("topic") == "/mission_target":
            self.mission_callback(data["msg"]["data"])

    def on_ws_error(self, ws, error):
        rospy.logerr(f"rosbridge 连接错误: {error}")

    def on_ws_close(self, ws, close_status_code, close_msg):
        rospy.logwarn("rosbridge 连接已断开。")

    def publish_via_websocket(self, topic_name, data_to_send):
        if not self.is_ws_connected:
            rospy.logerr(f"发布失败，rosbridge 未连接！")
            return
        if isinstance(data_to_send, int): msg_type = "std_msgs/Int32"; message_data = {"data": data_to_send}
        elif isinstance(data_to_send, str): msg_type = "std_msgs/String"; message_data = {"data": data_to_send}
        else: rospy.logerr(f"不支持的数据类型: {type(data_to_send)}"); return
        publish_message = { "op": "publish", "topic": topic_name, "msg": message_data, "type": msg_type }
        try:
            self.ws_app.send(json.dumps(publish_message))
            rospy.loginfo(f"通过 rosbridge 发布到话题 '{topic_name}': {message_data}")
        except Exception as e:
            rospy.logerr(f"通过 rosbridge 发布时出错: {e}")

    def mission_callback(self, new_id):
        if self.target_category_id != new_id:
            rospy.loginfo(f"接收到新任务！目标大类ID: {new_id}")
            self.target_category_id = new_id
            self.found_category_id = -1
            self.found_target_location = None
    
    def vision_callback(self, msg):
        if self.target_category_id == -1:
            return
        
        self.current_detections = []
        num_detections = int(msg.data[0])

        for i in range(num_detections):
            item_id = int(msg.data[1 + i * 6])
            if item_id in self.ITEM_TO_CATEGORY_MAP:
                if self.ITEM_TO_CATEGORY_MAP[item_id] == self.target_category_id:
                    self.current_detections.append(item_id)

    def has_target_in_view(self):
        return len(self.current_detections) > 0

    def stop_robot(self):
        self.move_base_client.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())

    def get_robot_pose(self):
        try:
            trans, _ = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

    def execute_recovery_maneuver(self):
        rospy.logwarn("检测到导航被卡住！正在执行自主解困机动...")
        self.stop_robot()
        rate = rospy.Rate(10)
        twist = Twist()

        rospy.loginfo("解困步骤1: 微量后退")
        twist.linear.x = self.RECOVERY_BACKWARD_DIST
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 1.0:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        rospy.loginfo("解困步骤2: 微量旋转")
        twist.linear.x = 0.0
        twist.angular.z = random.choice([self.RECOVERY_TURN_ANGLE, -self.RECOVERY_TURN_ANGLE])
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 1.5:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        rospy.loginfo("解困机动完成，将重新尝试导航。")
        rospy.sleep(0.5)

    def opportunistic_navigate(self, area_definition, area_name):
        rospy.loginfo(f"开始向区域 '{area_name}' 进行机会主义导航...")
        
        center_x = (area_definition['x'][0] + area_definition['x'][1]) / 2.0
        center_y = (area_definition['y'][0] + area_definition['y'][1]) / 2.0
        target_yaw = area_definition['yaw']
        goal = MoveBaseGoal(target_pose=PoseStamped(header=rospy.Header(frame_id="map", stamp=rospy.Time.now()),
                                                  pose=Pose(position=Point(center_x, center_y, 0),
                                                            orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, target_yaw)))))
        self.move_base_client.send_goal(goal)
        
        rate = rospy.Rate(self.MONITOR_RATE)
        start_time = rospy.Time.now()
        last_pos_time = rospy.Time.now()
        last_pos = self.get_robot_pose()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > self.NAVIGATION_TIMEOUT:
                rospy.logerr(f"导航至区域 '{area_name}' 超时！")
                self.stop_robot(); return False
            
            nav_state = self.move_base_client.get_state()
            if nav_state not in [actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                break
            
            current_pos = self.get_robot_pose()
            if current_pos[0] is not None:
                if (area_definition['x'][0] <= current_pos[0] <= area_definition['x'][1] and
                    area_definition['y'][0] <= current_pos[1] <= area_definition['y'][1]):
                    rospy.loginfo(f"机会主义导航成功！在途中已进入区域 '{area_name}'。")
                    self.stop_robot(); return True
                
                if last_pos[0] is not None and math.hypot(current_pos[0] - last_pos[0], current_pos[1] - last_pos[1]) > 0.02:
                    last_pos = current_pos
                    last_pos_time = rospy.Time.now()
                
                if (rospy.Time.now() - last_pos_time).to_sec() > self.STUCK_TIMEOUT:
                    self.execute_recovery_maneuver()
                    self.move_base_client.send_goal(goal)
                    start_time = rospy.Time.now()
                    last_pos_time = rospy.Time.now()
            rate.sleep()

        current_pos = self.get_robot_pose()
        if current_pos[0] is not None and (area_definition['x'][0] <= current_pos[0] <= area_definition['x'][1] and area_definition['y'][0] <= current_pos[1] <= area_definition['y'][1]):
            return True
        
        rospy.logerr(f"导航至区域 '{area_name}' 彻底失败。")
        return False

    def peek_for_target(self):
        rospy.loginfo("开始原地旋转观察..."); twist = Twist(); rate = rospy.Rate(20)
        for direction in [1, -1]:
            twist.angular.z = self.SEARCH_ROTATION_SPEED * direction
            start_time = rospy.Time.now()
            turn_duration_secs = self.PEEK_ANGLE_RANGE / self.SEARCH_ROTATION_SPEED
            if direction == -1: turn_duration_secs *= 2
            
            while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < turn_duration_secs:
                self.cmd_vel_pub.publish(twist)
                if self.has_target_in_view():
                    if self.current_detections:
                        self.found_item_id = self.current_detections[0]
                        self.found_item_name = self.ITEM_ID_TO_NAME_MAP.get(self.found_item_id, "unknown_item")
                        rospy.loginfo(f"观察中发现目标物品: ID={self.found_item_id}, Name='{self.found_item_name}'")
                    self.stop_robot(); return True
                rate.sleep()
        self.stop_robot(); rospy.loginfo("观察结束，此区域未发现目标。"); return False

    def report_mission_results(self):
        rospy.loginfo("===== 任务总结与通信阶段 =====")
        rospy.sleep(1.0)
        
        if self.found_item_name and self.found_target_location:
            self.publish_via_websocket("/sim_item_result", self.found_item_name)
            rospy.sleep(0.1)
            self.publish_via_websocket("/sim_room_result", self.found_target_location)
        else:
            rospy.logwarn("未找到任何具体物品或位置，不发送通信结果。")
            
        rospy.loginfo("通信阶段完成。")

    def run(self):
        rospy.loginfo_throttle(5, "正在等待任务指令 (通过rosbridge)...")
        while self.target_category_id == -1 and not rospy.is_shutdown():
            rospy.sleep(0.5)

        rospy.loginfo(f"任务开始！正在搜索目标大类ID: {self.target_category_id}")
        for room_name in self.ROOM_SEARCH_AREAS.keys():
            if rospy.is_shutdown() or (rospy.Time.now() - self.mission_start_time) > self.simulation_timeout:
                rospy.logerr("任务被中断或总超时！"); break
            
            if self.opportunistic_navigate(self.ROOM_SEARCH_AREAS[room_name], f"房间 {room_name}"):
                rospy.sleep(0.2)
                if self.peek_for_target():
                    if self.found_target_location is None:
                        rospy.loginfo(f"已在房间 '{room_name}' 首次记录到目标！")
                        self.found_target_location = room_name
                        self.found_category_id = self.target_category_id
            else:
                rospy.logwarn(f"未能进入房间 '{room_name}'，立即无缝前往下一个。")
        
        rospy.loginfo("===== 所有房间巡视完毕，开始返航 =====")
        # [核心修改] 返航时调用“机会主义导航”进入安全区
        if self.opportunistic_navigate(self.HOME_SAFE_AREA, "返航安全区"):
            rospy.loginfo("已成功返回出发区范围。")
            self.report_mission_results()
        else:
            rospy.logerr("返航失败！任务结果可能无法发送！")

        rospy.loginfo("所有流程结束。")
        if self.ws_app:
            self.ws_app.close()

if __name__ == '__main__':
    try:
        controller = TaskController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务控制器被中断。")