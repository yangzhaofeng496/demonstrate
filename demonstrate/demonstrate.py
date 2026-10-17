#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import csv
import math
import time
import threading
from dobot_v4_bringup.msg import ToolVectorActual, RobotStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import JointState, Joy  # 新增：导入 Joy 消息类型
# ROS 1 通常使用 tf.transformations 处理四元数
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Trigger, TriggerResponse
from tf2_geometry_msgs import PoseStamped

class DemonstrateTemplateNode:
    def __init__(self):
        # 1. 初始化 ROS 1 节点
        rospy.init_node('demonstrate_node', anonymous=False)

        # 2. TF2 Buffer 和 Listener (ROS 1 中不需要传 self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.record_cartesian = rospy.get_param('~record_cartesian', True)
        if not isinstance(self.record_cartesian, bool):
            raise ValueError('record_cartesian must be true or false')
        self.record_lock = threading.RLock()
        self.tool_pose_snapshot = None
        self.robot_status_snapshot = (False, None)

        self.file_path = rospy.get_param(
            '~output_file',
            '/home/shansu/moma_ws/src/demonstrate/path/HXD1C_test.csv')
        if not isinstance(self.file_path, str) or not self.file_path:
            raise ValueError('output_file must be a non-empty path string')
        self.file = open(self.file_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            '项点编号', '底盘x', '底盘y', '底盘yaw角度',
            '关节1', '关节2', '关节3', '关节4', '关节5', '关节6',
            '检测类型', '项点类型', '报活处理班组', '架次',
            '方位', '左右位', '位别', '项点名称'
        ] + (['tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
              'tcp_rx_deg', 'tcp_ry_deg', 'tcp_rz_deg']
             if self.record_cartesian else []))
        self.file.flush()  # 确保数据立即写入文件
        self.chas_x = 0.0
        self.chas_y = 0.0
        self.chas_yaw = 0.0
        self.joint_angles = [0.00] * 6 
        self.cnt = 0
        self.last_button_state = 1  # 初始状态假设为按下（1），这样第一次按下时会触发上升沿检测
        self.last_transition_button_state = 0

        # ROS 1 的延时等待
        rospy.sleep(1.0) 
        try:
            # ROS 1 中 Time(0) 表示最新的变换，Duration 直接传对象
            self.tf_buffer.lookup_transform('base_link', 'Link6', rospy.Time(0), rospy.Duration(3.0))
        except Exception as e:
            rospy.logwarn(f'Could not get initial transform: {e}')
        
        self.joy_subscriber = rospy.Subscriber(
            '/joy_teleop/joy', 
            Joy, 
            self.joy_callback
        )

        # 3. 订阅器语法修改
        self.odom_subscriber = rospy.Subscriber(
            '/Odometry',
            Odometry,
            self.odom_callback
        )
        
        self.joint_subscriber = rospy.Subscriber(
            '/joint_states', 
            JointState, 
            self.joint_states_callback
        )

        
        if self.record_cartesian:
            self.tool_pose_subscriber = rospy.Subscriber(
                '/dobot_v4_bringup/msg/ToolVectorActual',
                ToolVectorActual, self.tool_pose_callback, queue_size=1)
            self.robot_status_subscriber = rospy.Subscriber(
                '/dobot_v4_bringup/msg/RobotStatus',
                RobotStatus, self.robot_status_callback, queue_size=1)

        self.record_service = rospy.Service(
        'trigger_record', 
        Trigger, 
        self.handle_trigger
    )

        rospy.on_shutdown(self.shutdown_hook)
    
        rospy.loginfo("记录服务 [trigger_record] 已启动")

    def joy_callback(self, msg):
        """
        手柄回调函数：实现上升沿检测
        """
        if len(msg.buttons) < 4:
            rospy.logwarn("手柄消息 buttons 列表长度不足")
            return
        # 列表第 4 位对应的索引是 3
        current = msg.buttons[3]
        transition = msg.buttons[1]  # 按钮2用于记录过渡点
        # 上升沿逻辑：当前是 1 (按下)，上一次是 0 (松开)
        record_edge = current == 1 and self.last_button_state == 0
        # 过渡点上升沿逻辑：当前是 1 (按下)，上一次是 0 (松开)
        transition_edge = transition == 1 and self.last_transition_button_state == 0
        # 关键：更新状态，为下一次回调做准备
        # 先更新按键状态，采样失败后按住按钮不会自动重复打点。
        self.last_button_state = current
        self.last_transition_button_state = transition

        if msg.buttons[2] == 1:  # 按钮3用于关闭节点
            # 关键：发送信号让 ROS 彻底关闭节点
            # 这样做会自动触发 self.shutdown_hook()
            rospy.signal_shutdown("User pressed shutdown button")
            return
        try:
            if record_edge:
                self.setItem_callback()
            if transition_edge:
                self.set_transition_item_callback()
        except Exception as e:
            rospy.logerr(f"手柄记录失败: {e}")

    def tool_pose_callback(self, msg):
        values = (msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz)
        with self.record_lock:
            self.tool_pose_snapshot = (
                (values, time.monotonic())
                if all(math.isfinite(v) for v in values) else None)

    def robot_status_callback(self, msg):
        with self.record_lock:
            self.robot_status_snapshot = (msg.is_connected, time.monotonic())
            if not msg.is_connected:
                self.tool_pose_snapshot = None

    def get_record_tool_pose(self):
        if not self.record_cartesian:
            return []
        with self.record_lock:
            snapshot = self.tool_pose_snapshot
            connected, status_at = self.robot_status_snapshot
        now = time.monotonic()
        if not connected or status_at is None or now - status_at > 1.0:
            raise RuntimeError("机械臂驱动未连接或状态接收超时")
        if snapshot is None or now - snapshot[1] > 0.5:
            raise RuntimeError("尚未收到有效机械臂位姿或位姿接收超时")
        return [f"{value:.6f}" for value in snapshot[0]]

    # def odom_callback(self, msg):
    #     try:
    #         pose_odom = PoseStamped()
    #         pose_odom.header = msg.header      # 沿用原消息时间戳+frame_id(一般是odom)
    #         pose_odom.pose = msg.pose.pose
            
    #         transform_pose = self.tf_buffer.transform(
    #             pose_odom,
    #             target_frame="map",
    #             timeout=rospy.Duration(0.2)
    #         )

    #         self.chas_x = transform_pose.pose.position.x
    #         self.chas_y = transform_pose.pose.position.y
    #         chas_q = transform_pose.pose.orientation
    #         map_quat = (chas_q.x, chas_q.y, chas_q.z, chas_q.w)
    #         (_, _, self.chas_yaw) = euler_from_quaternion(map_quat)

    #     except Exception as e:
    #         rospy.logerr(f'Error in odom_callback: {e}')

    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                msg.header.stamp,
                rospy.Duration(0.2)
            )

            self.chas_x = transform.transform.translation.x
            self.chas_y = transform.transform.translation.y
            q = transform.transform.rotation
            (_, _, self.chas_yaw) = euler_from_quaternion(
                (q.x, q.y, q.z, q.w)
            )
        except Exception as e:
            rospy.logerr(f'Error getting map->base_link: {e}')

    def joint_states_callback(self, msg):
        try:
            # msg.position 是一个包含所有关节角度的元组
            # 注意：msg.position 的顺序是对应 msg.name 里的顺序的
            if len(msg.position) >= 6:
                # 方案 A：直接保存整个列表
                self.joint_angles = list(msg.position)
        except Exception as e:
            rospy.logerr(f"获取关节角失败: {e}")
        
        
    def handle_trigger(self, req):
        """
        服务回调函数
        """
        try:
            # 执行原来的记录逻辑
            item_number = self.setItem_callback()
            
            # 返回成功响应
            return TriggerResponse(
                success=True,
                message=f"成功记录第 {item_number} 组数据"
            )
        except Exception as e:
            # 返回失败响应
            return TriggerResponse(
                success=False,
                message=f"记录失败: {str(e)}"
            )

    def setItem_callback(self):
        return self.write_record(transition=False)

    def set_transition_item_callback(self):
        # 过渡点固定编号为 0，记录底盘位姿和关节角；项点名称置为 TEMP
        return self.write_record(transition=True)

    def write_record(self, transition):
        with self.record_lock:
            tool_pose = self.get_record_tool_pose()
            item_number = 0 if transition else self.cnt + 1
            # 只记录前 6 个关节角，并转换为角度值
            joint_values = [f"{(angle * 57.3):.3f}" for angle in self.joint_angles[:6]]
            if len(joint_values) < 6:
                joint_values += [""] * (6 - len(joint_values))
            # 以下说明对应原来的 18 列：
            # 示教阶段仅写入前 10 列，其余业务列留空占位
            # 开启笛卡尔记录时，在末尾追加 TCP 位姿；过渡点项点名称置为 TEMP。
            row = [
                item_number,
                f"{self.chas_x:.3f}", f"{self.chas_y:.3f}", f"{self.chas_yaw:.3f}",
                *joint_values,
                "", "", "", "", "", "", "", "TEMP" if transition else ""
            ] + tool_pose
            self.writer.writerow(row)
            self.file.flush()  # 确保数据立即写入文件
            if not transition:
                self.cnt = item_number
            rospy.loginfo("成功写入过渡点" if transition else f"成功写入第{item_number}项")
            return item_number

    def shutdown_hook(self):
        with self.record_lock:
            rospy.loginfo("Shutting down, closing file...")
            self.file.close()

if __name__ == '__main__':
    try:
        node = DemonstrateTemplateNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass