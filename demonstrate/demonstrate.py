#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import csv
import math
import os
import threading
import time

import numpy as np
import rospy
import tf2_ros
from dobot_v4_bringup.msg import RobotStatus, ToolVectorActual
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState, Joy
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_matrix

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from columns import TRANSITION_POINT_NAME, header
    

try:
    from moma_light.msg import LightState
except ImportError as import_error:  # 光源包未编译时只关闭光源记录
    LightState = None
    LIGHT_IMPORT_ERROR = import_error
else:
    LIGHT_IMPORT_ERROR = None

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError as import_error:  # 缺少图像依赖时仍允许示教，只是不拍照
    cv2 = None
    CvBridge = None
    IMAGE_IMPORT_ERROR = import_error
else:
    IMAGE_IMPORT_ERROR = None

# 光轴与重力夹角小于约 1 度时，重力在像平面内的投影可忽略，角度无意义。
IMAGE_ROTATION_MIN_SIN = np.sin(np.radians(1.0))


def image_rotation_deg(rotation):
    """计算让图像“重力朝下”需绕相机光轴旋转的角度，单位度。

    ``rotation`` 是相机光学系在 map 系中的旋转矩阵（numpy 3x3 或 4x4，
    由 ``tf.transformations.quaternion_matrix`` 直接得到），只用到第三行：
    它等于 ``Rᵀ·(0,0,-1)``，即重力在相机系中的表示。

    正值表示绕 +Z 光轴（指向场景）按右手定则旋转相机。
    光轴与重力平行时返回 ``None``。
    """
    gravity_x, gravity_y = -rotation[2, 0], -rotation[2, 1]
    if np.hypot(gravity_x, gravity_y) < IMAGE_ROTATION_MIN_SIN:
        return None
    return float(np.degrees(np.arctan2(-gravity_x, gravity_y)))


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
        # 串行化整行写入，避免手柄与服务同时触发时交错写文件。
        self.write_lock = threading.Lock()
        self.tool_pose_snapshot = None
        self.robot_status_snapshot = (False, None)

        # 位姿与坐标系列参数
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.tf_timeout = float(rospy.get_param('~tf_timeout', 0.2))
        if not isinstance(self.map_frame, str) or not self.map_frame:
            raise ValueError('map_frame must be a non-empty string')
        if not isinstance(self.base_frame, str) or not self.base_frame:
            raise ValueError('base_frame must be a non-empty string')

        # 末端与相机的朝向都直接查 TF，不自行拼接或换算外参
        self.ee_frame = str(rospy.get_param('~ee_frame', 'Link6'))
        self.camera_frame = str(rospy.get_param('~camera_frame', ''))

        # 图像参数
        self.image_topic = str(rospy.get_param('~image_topic', '/camera/image_raw'))
        self.image_encoding = str(rospy.get_param('~image_encoding', 'bgr8'))
        self.image_max_age = float(rospy.get_param('~image_max_age', 0.5))
        self.image_format = str(rospy.get_param('~image_format', 'png')).lstrip('.')
        self.require_image = bool(rospy.get_param('~require_image', False))
        self.image_msg = None
        self.image_at = None

        # 光源参数：只被动订阅 moma_light 的状态话题，当前仅记录白光通道
        self.light_state_topic = str(
            rospy.get_param('~light_state_topic', '/light/state'))
        self.light_white = None  # (on: bool, brightness: int 0-255)

        # 过渡点参数
        self.transition_record_base = bool(
            rospy.get_param('~transition_record_base', False))
        self.transition_base_warn_distance = float(
            rospy.get_param('~transition_base_warn_distance', 0.05))
        self.transition_base_warn_angle = math.radians(float(
            rospy.get_param('~transition_base_warn_angle_deg', 3.0)))

        self.file_path = self.resolve_output_path()
        output_dir = os.path.dirname(self.file_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        self.file = open(self.file_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        # 表头统一由 columns.header 生成，避免与离线校验脚本各维护一份拼接顺序
        self.writer.writerow(header(self.record_cartesian))
        self.file.flush()  # 确保数据立即写入文件

        image_dir = str(rospy.get_param('~image_dir', ''))
        if not image_dir:
            stem, _ = os.path.splitext(self.file_path)
            image_dir = stem + '_images'
        self.image_dir = image_dir

        self.chas_x = 0.0
        self.chas_y = 0.0
        self.chas_yaw = 0.0
        self.joint_angles = [0.00] * 6
        self.cnt = 0
        self.transition_cnt = 0
        self.last_base_pose = None
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

        # 无论是否记录笛卡尔列都订阅，末端位姿需要同一来源。
        self.tool_pose_subscriber = rospy.Subscriber(
            '/dobot_v4_bringup/msg/ToolVectorActual',
            ToolVectorActual, self.tool_pose_callback, queue_size=1)
        self.robot_status_subscriber = rospy.Subscriber(
            '/dobot_v4_bringup/msg/RobotStatus',
            RobotStatus, self.robot_status_callback, queue_size=1)

        if self.image_topic:
            if CvBridge is None:
                rospy.logerr('图像依赖缺失（%s），本次不拍照', IMAGE_IMPORT_ERROR)
                self.image_topic = ''
            else:
                self.bridge = CvBridge()
                self.image_subscriber = rospy.Subscriber(
                    self.image_topic, Image, self.image_callback, queue_size=1)

        if self.light_state_topic:
            if LightState is None:
                rospy.logerr(
                    'moma_light 不可导入（%s），本次不记录光源', LIGHT_IMPORT_ERROR)
                self.light_state_topic = ''
            else:
                rospy.Subscriber(
                    self.light_state_topic, LightState,
                    self.light_state_callback, queue_size=1)

        self.record_service = rospy.Service(
            'trigger_record',
            Trigger,
            self.handle_trigger
        )

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("记录服务 [trigger_record] 已启动")
        self.warn_missing_sources()

    @staticmethod
    def resolve_output_path():
        """解析示教 CSV 的完整路径。

        优先使用 ``~output_file``（整路径覆盖），否则由 ``~output_dir``
        与 ``~output_file_name`` 拼出；两者都没配置时直接报错，
        避免静默写到意料之外的位置。
        """
        override = rospy.get_param('~output_file', '')
        if isinstance(override, str) and override.strip():
            return override
        output_dir = rospy.get_param('~output_dir', '')
        file_name = rospy.get_param('~output_file_name', '')
        if not isinstance(output_dir, str) or not isinstance(file_name, str):
            raise ValueError('output_dir 与 output_file_name 必须是字符串')
        if not file_name.strip():
            raise ValueError(
                '未配置示教输出文件：请在配置中填写 output_dir 与 output_file_name')
        return os.path.join(output_dir or '.', file_name.strip())

    def warn_missing_sources(self):
        """启动时提示未配置的来源，避免现场误以为记录成功。"""
        if not self.ee_frame:
            rospy.logwarn('未配置 ee_frame，末端 map 位姿列将留空')
        if not self.camera_frame:
            rospy.logwarn('未配置 camera_frame，image_rotation_deg 列将留空')

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

    def image_callback(self, msg):
        with self.record_lock:
            self.image_msg = msg
            self.image_at = time.monotonic()

    def light_state_callback(self, msg):
        """缓存白光通道状态；LightState 只按颜色名暴露，不出现通道号。"""
        with self.record_lock:
            self.light_white = (bool(msg.white.on), int(msg.white.brightness))

    def read_tool_pose_snapshot(self):
        """读取机械臂 TCP 快照，返回 ``(values, error)``。

        ``values`` 为 ``(x, y, z, rx, ry, rz)``，控制器原始单位（毫米/度）；
        不可用时为 ``None``，``error`` 说明原因。
        """
        with self.record_lock:
            snapshot = self.tool_pose_snapshot
            connected, status_at = self.robot_status_snapshot
        now = time.monotonic()
        if not connected or status_at is None or now - status_at > 1.0:
            return None, "机械臂驱动未连接或状态接收超时"
        if snapshot is None or now - snapshot[1] > 0.5:
            return None, "尚未收到有效机械臂位姿或位姿接收超时"
        return snapshot[0], None

    def lookup_transform(self, target_frame, source_frame):
        """查询 ``target <- source`` 的最新变换，失败返回 None。

        TF 树本身已经包含了定位、里程计与 URDF 的融合结果，
        因此这里直接取结果，不再自行做任何拼接。
        """
        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0),
                rospy.Duration(self.tf_timeout))
        except Exception as e:
            rospy.logwarn_throttle(
                5.0, 'TF 查询失败 %s <- %s: %s', target_frame, source_frame, e)
            return None

    # def odom_callback(self, msg):
    #     try:
    #         pose_odom = PoseStamped()
    #         pose_odom.header = msg.header      # 沿用原消息时间戳+frame_id(一般是odom)
    #         pose_odom.pose = msg.pose.pose
    #
    #         transform_pose = self.tf_buffer.transform(
    #             pose_odom,
    #             target_frame="map",
    #             timeout=rospy.Duration(0.2)
    #         )
    #
    #         self.chas_x = transform_pose.pose.position.x
    #         self.chas_y = transform_pose.pose.position.y
    #         chas_q = transform_pose.pose.orientation
    #         map_quat = (chas_q.x, chas_q.y, chas_q.z, chas_q.w)
    #         (_, _, self.chas_yaw) = euler_from_quaternion(map_quat)
    #
    #     except Exception as e:
    #         rospy.logerr(f'Error in odom_callback: {e}')

    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
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
        # 过渡点使用负数编号，记录底盘位姿和关节角；项点名称置为 TEMP
        return self.write_record(transition=True)

    def resolve_base_pose(self, transition, live_pose, previous_pose):
        """确定本行写入的底盘位姿。

        过渡点默认沿用上一个正式项点的底盘位姿（底盘不动），
        便于巡逻节点按“距离为零”自动跳过底盘运动。
        """
        if not transition or self.transition_record_base:
            return live_pose
        if previous_pose is None:
            rospy.logwarn(
                '过渡点之前没有正式项点，无法沿用底盘位姿，本次记录实时值')
            return live_pose
        distance = math.hypot(
            live_pose[0] - previous_pose[0], live_pose[1] - previous_pose[1])
        delta_yaw = math.atan2(
            math.sin(live_pose[2] - previous_pose[2]),
            math.cos(live_pose[2] - previous_pose[2]))
        if (distance > self.transition_base_warn_distance
                or abs(delta_yaw) > self.transition_base_warn_angle):
            rospy.logwarn_throttle(
                5.0,
                '过渡点按配置不记录底盘移动，但实时底盘与上一项点相差 %.3f m / %.1f 度；'
                '如需记录，请把 transition_record_base 设为 true',
                distance, math.degrees(delta_yaw))
        return previous_pose

    def compute_ee_pose(self):
        """末端在 map 下的位姿 ``(label, (x, y, z, rx, ry, rz))``。

        数据直接取自 TF，位置为米、角度为弧度；不可用时返回 ``(None, None)``。
        """
        if not self.ee_frame:
            return None, None
        transform = self.lookup_transform(self.map_frame, self.ee_frame)
        if transform is None:
            return None, None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        roll, pitch, yaw = euler_from_quaternion(
            (rotation.x, rotation.y, rotation.z, rotation.w))
        return 'tf:' + self.ee_frame, (
            translation.x, translation.y, translation.z, roll, pitch, yaw)

    def compute_image_rotation(self):
        """返回让图像重力朝下需绕相机光轴旋转的角度（度），不可用时为 None。"""
        if not self.camera_frame:
            return None
        transform = self.lookup_transform(self.map_frame, self.camera_frame)
        if transform is None:
            return None
        rotation = transform.transform.rotation
        matrix = quaternion_matrix(
            (rotation.x, rotation.y, rotation.z, rotation.w))
        return image_rotation_deg(matrix)

    def capture_image(self, item_number):
        """把最新相机帧保存为 ``<image_dir>/<item_number>.<ext>``，返回文件名。"""
        if not self.image_topic:
            return ''
        with self.record_lock:
            msg = self.image_msg
            received_at = self.image_at
        if msg is None or received_at is None:
            message = '尚未收到相机图像，跳过拍照'
        elif time.monotonic() - received_at > self.image_max_age:
            message = '相机图像已超时，跳过拍照'
        else:
            message = None
        if message is not None:
            if self.require_image:
                raise RuntimeError(message)
            rospy.logwarn_throttle(5.0, '%s（话题 %s）', message, self.image_topic)
            return ''
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding=self.image_encoding)
        except Exception as e:
            rospy.logerr(f'相机图像转换失败: {e}')
            if self.require_image:
                raise
            return ''
        os.makedirs(self.image_dir, exist_ok=True)
        file_name = f'{item_number}.{self.image_format}'
        file_path = os.path.join(self.image_dir, file_name)
        if os.path.exists(file_path):
            rospy.logwarn('图像已存在，将被覆盖: %s', file_path)
        if not cv2.imwrite(file_path, frame):
            rospy.logerr('图像写入失败: %s', file_path)
            if self.require_image:
                raise RuntimeError('图像写入失败')
            return ''
        rospy.loginfo('图像已保存: %s', file_path)
        return file_name

    def write_record(self, transition):
        with self.write_lock:
            tool_values, tool_error = self.read_tool_pose_snapshot()
            with self.record_lock:
                previous_pose = self.last_base_pose
                live_pose = (self.chas_x, self.chas_y, self.chas_yaw)
                light_white = self.light_white

            tool_pose = []
            if self.record_cartesian:
                if tool_values is None:
                    # 保持原有行为：需要笛卡尔列时，位姿不可用则整行不写
                    raise RuntimeError(tool_error)
                tool_pose = [f"{value:.6f}" for value in tool_values]

            # 过渡点用负数编号，从 -1 开始递减，与正式项点编号互不干扰
            if transition:
                item_number = self.transition_cnt - 1
            else:
                item_number = self.cnt + 1

            base_x, base_y, base_yaw = self.resolve_base_pose(
                transition, live_pose, previous_pose)

            # 只记录前 6 个关节角，并转换为角度值
            joint_values = [f"{(angle * 57.3):.3f}" for angle in self.joint_angles[:6]]
            if len(joint_values) < 6:
                joint_values += [""] * (6 - len(joint_values))

            ee_label, ee_values = self.compute_ee_pose()
            ee_pose = [] if ee_values is None else [
                f'{value:.6f}' for value in ee_values]
            rotation_deg = self.compute_image_rotation()
            # 过渡点只摆臂、不检测，因此不拍照
            image_name = '' if transition else self.capture_image(item_number)

            ext_row = [
                self.map_frame,
                ee_label or '',
                *(ee_pose or [''] * 6),
                image_name,
                '' if rotation_deg is None else f'{rotation_deg:.3f}',
                '' if light_white is None else str(light_white[1]),
                '' if light_white is None else ('true' if light_white[0] else 'false'),
            ]

            # 以下说明对应原来的 18 列：
            # 示教阶段仅写入前 10 列，其余业务列留空占位
            # 开启笛卡尔记录时，在末尾追加 TCP 位姿；过渡点项点名称置为 TEMP。
            row = [
                item_number,
                f"{base_x:.3f}", f"{base_y:.3f}", f"{base_yaw:.3f}",
                *joint_values,
                "", "", "", "", "", "", "",
                TRANSITION_POINT_NAME if transition else ""
            ] + tool_pose + ext_row
            self.writer.writerow(row)
            self.file.flush()  # 确保数据立即写入文件
            # 计数在整行写入成功后才提交，异常时不会跳号
            if transition:
                self.transition_cnt = item_number
            else:
                self.cnt = item_number
                self.last_base_pose = (base_x, base_y, base_yaw)
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
