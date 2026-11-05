#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""write_record 的离线集成测试：用桩件替换 ROS，真实写出一份 CSV 再校验。

覆盖点：
* 原有 18(+6) 列的名称、顺序与数值格式不变；
* 末端位姿直接来自 TF，缺 TF 时留空而不报错；
* 过渡点负编号、底盘沿用上一正式项点；
* 拍照文件名与光源列；
* 图像旋转角的已知取值与退化判定。

运行：
    python3 test/test_write_record.py
"""

import csv
import math
import os
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
MODULE_DIR = os.path.join(ROOT, '..', 'demonstrate')
sys.path.insert(0, MODULE_DIR)
sys.path.insert(0, ROOT)

import check_csv_compat  # noqa: E402  pylint: disable=wrong-import-position
import columns  # noqa: E402  pylint: disable=wrong-import-position


# --------------------------------------------------------------------------
# 桩件：在导入节点模块之前注入 sys.modules
# --------------------------------------------------------------------------

class _Vector3(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class _Quaternion(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class Stub(object):
    params = {}
    subscribers = {}
    transforms = {}
    written_images = {}


def _quaternion_from_rpy(rx_deg, ry_deg, rz_deg):
    """固定轴 X-Y-Z（R = Rz·Ry·Rx）欧拉角转四元数。"""
    a, b, c = (math.radians(v) / 2.0 for v in (rx_deg, ry_deg, rz_deg))
    cr, sr = math.cos(a), math.sin(a)
    cp, sp = math.cos(b), math.sin(b)
    cy, sy = math.cos(c), math.sin(c)
    return _Quaternion(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy)


class _Stamped(object):
    pass


def _transform(x=0.0, y=0.0, z=0.0, rpy=(0.0, 0.0, 0.0)):
    stamped = _Stamped()
    stamped.transform = _Stamped()
    stamped.transform.translation = _Vector3(x, y, z)
    stamped.transform.rotation = _quaternion_from_rpy(*rpy)
    return stamped


def _light_state(white_on=True, white_rate=0, red_rate=255, green_on=True):
    """构造 moma_light/LightState：颜色为第一级，开关/亮度为第二级。"""
    def channel(on, brightness):
        return types.SimpleNamespace(on=on, brightness=brightness)
    return types.SimpleNamespace(
        red=channel(True, red_rate),
        green=channel(green_on, 64),
        blue=channel(True, 32),
        white=channel(white_on, white_rate))


def _rotation_matrix(rx_deg, ry_deg, rz_deg):
    """R = Rz·Ry·Rx，用于旋转角的第一性验证，不依赖桩件。"""
    rx, ry, rz = (math.radians(v) for v in (rx_deg, ry_deg, rz_deg))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ])


def _install_stubs():
    class Time(object):
        def __init__(self, secs=0, nsecs=0):
            self.secs, self.nsecs = secs, nsecs

    class Duration(object):
        def __init__(self, secs=0, nsecs=0):
            self.secs, self.nsecs = secs, nsecs

    class ROSException(Exception):
        pass

    class Subscriber(object):
        def __init__(self, topic, msg_type, callback, **kwargs):
            Stub.subscribers.setdefault(topic, []).append(callback)

    def noop_log(level):
        def inner(message, *args):
            pass
        return inner

    rospy = types.ModuleType('rospy')
    rospy.init_node = lambda name, anonymous=False: None
    rospy.get_param = lambda name, default=None: Stub.params.get(
        name.lstrip('~'), default)
    rospy.sleep = lambda seconds: None
    rospy.Subscriber = Subscriber
    rospy.Service = lambda name, srv_type, callback: None
    rospy.on_shutdown = lambda callback: None
    rospy.signal_shutdown = lambda reason=None: None
    rospy.Time, rospy.Duration = Time, Duration
    rospy.ROSException = ROSException
    rospy.ROSInterruptException = ROSException
    for level in ('loginfo', 'logwarn', 'logerr',
                  'logwarn_throttle', 'logerr_throttle', 'loginfo_throttle'):
        setattr(rospy, level, noop_log(level))
    sys.modules['rospy'] = rospy

    class Buffer(object):
        def lookup_transform(self, target, source, time=None, timeout=None):
            transform = Stub.transforms.get((target, source))
            if transform is None:
                raise ROSException('no transform %s <- %s' % (target, source))
            return transform

    tf2_ros = types.ModuleType('tf2_ros')
    tf2_ros.Buffer = Buffer
    tf2_ros.TransformListener = lambda buffer: None
    sys.modules['tf2_ros'] = tf2_ros

    def euler_from_quaternion(quaternion, axes='sxyz'):
        x, y, z, w = quaternion
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def quaternion_matrix(quaternion):
        x, y, z, w = quaternion
        norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        matrix = np.identity(4)
        matrix[0, :3] = [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                         2 * (x * z + y * w)]
        matrix[1, :3] = [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                         2 * (y * z - x * w)]
        matrix[2, :3] = [2 * (x * z - y * w), 2 * (y * z + x * w),
                         1 - 2 * (x * x + y * y)]
        return matrix

    tf_pkg = types.ModuleType('tf')
    transformations = types.ModuleType('tf.transformations')
    transformations.euler_from_quaternion = euler_from_quaternion
    transformations.quaternion_matrix = quaternion_matrix
    tf_pkg.transformations = transformations
    sys.modules['tf'] = tf_pkg
    sys.modules['tf.transformations'] = transformations

    class _Msg(object):
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)

    for package, attrs in (
            ('sensor_msgs', {}),
            ('sensor_msgs.msg', {'Image': _Msg, 'JointState': _Msg,
                                 'Joy': _Msg}),
            ('nav_msgs', {}),
            ('nav_msgs.msg', {'Odometry': _Msg}),
            ('std_msgs', {}),
            ('std_msgs.msg', {}),
            ('std_srvs', {}),
            ('std_srvs.srv', {'Trigger': lambda: None,
                              'TriggerResponse': lambda **kw: kw}),
            ('dobot_v4_bringup', {}),
            ('dobot_v4_bringup.msg', {'ToolVectorActual': _Msg,
                                      'RobotStatus': _Msg}),
            ('moma_light', {}),
            ('moma_light.msg', {'LightState': _Msg,
                                'LightChannel': _Msg})):
        module = types.ModuleType(package)
        for key, value in attrs.items():
            setattr(module, key, value)
        sys.modules[package] = module

    class CvBridge(object):
        def imgmsg_to_cv2(self, msg, desired_encoding=None):
            return msg.payload

    def imwrite(path, frame):
        Stub.written_images[path] = frame
        with open(path, 'wb') as handle:
            handle.write(b'stub')
        return True

    cv_bridge = types.ModuleType('cv_bridge')
    cv_bridge.CvBridge = CvBridge
    sys.modules['cv_bridge'] = cv_bridge
    cv2 = types.ModuleType('cv2')
    cv2.imwrite = imwrite
    sys.modules['cv2'] = cv2


_install_stubs()
import demonstrate  # noqa: E402  pylint: disable=wrong-import-position


class WriteRecordTest(unittest.TestCase):
    def setUp(self):
        output_dir = tempfile.mkdtemp()
        Stub.params = {
            'record_cartesian': True,
            'output_dir': output_dir,
            'output_file_name': 'teach.csv',
            'output_file': '',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'tf_timeout': 0.2,
            'ee_frame': 'Link6',
            'camera_frame': '',
            'image_topic': '/camera/image_raw',
            'image_encoding': 'bgr8',
            'image_max_age': 0.5,
            'image_format': 'png',
            'require_image': False,
            'light_state_topic': '/light/state',
            'transition_record_base': False,
            'transition_base_warn_distance': 0.05,
            'transition_base_warn_angle_deg': 3.0,
        }
        Stub.subscribers = {}
        Stub.transforms = {('map', 'base_link'): _transform(1.0, 2.0, 0.5)}
        Stub.written_images = {}
        self.csv_path = os.path.join(output_dir, 'teach.csv')
        self.node = demonstrate.DemonstrateTemplateNode()
        self.addCleanup(self._close)

    def _close(self):
        try:
            self.node.file.close()
        except Exception:
            pass

    def new_node(self, **params):
        """改参数后重建节点；新节点的订阅会成为最新回调。"""
        self._close()
        defaults = {'output_dir': tempfile.mkdtemp(), 'output_file_name': 'teach.csv',
                    'output_file': ''}
        defaults.update(params)
        Stub.params = dict(Stub.params, **defaults)
        override = Stub.params['output_file']
        self.csv_path = override or os.path.join(
            Stub.params['output_dir'], Stub.params['output_file_name'])
        self.node = demonstrate.DemonstrateTemplateNode()
        return self.node

    # -- 辅助 -------------------------------------------------------------
    def send(self, topic, message):
        Stub.subscribers[topic][-1](message)

    def feed_arm(self):
        """只喂机械臂状态：笛卡尔列要求位姿可用。"""
        self.send('/dobot_v4_bringup/msg/RobotStatus',
                  types.SimpleNamespace(is_connected=True))
        self.send('/dobot_v4_bringup/msg/ToolVectorActual',
                  types.SimpleNamespace(x=100.0, y=0.0, z=0.0,
                                        rx=0.0, ry=0.0, rz=0.0))
        self.send('/joint_states',
                  types.SimpleNamespace(
                      position=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], name=[]))

    def feed_sensors(self):
        self.feed_arm()
        self.send('/camera/image_raw',
                  types.SimpleNamespace(payload=b'fake-frame'))
        self.send('/light/state', _light_state(white_on=True, white_rate=128))

    def feed_odom(self, x, y, yaw_deg=0.0):
        Stub.transforms[('map', 'base_link')] = _transform(
            x, y, 0.5, (0.0, 0.0, yaw_deg))
        self.send('/Odometry', types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=0.0, frame_id='odom')))

    def read_csv(self):
        self.node.file.close()
        with open(self.csv_path, 'r', newline='', encoding='utf-8') as handle:
            rows = list(csv.reader(handle))
        return rows[0], rows[1:]

    # -- 输出路径 ---------------------------------------------------------
    def test_csv_path_from_output_dir_and_name(self):
        header, _ = self.read_csv()
        self.assertTrue(os.path.isfile(self.csv_path))
        self.assertEqual(self.csv_path,
                         os.path.join(Stub.params['output_dir'], 'teach.csv'))
        self.assertEqual(header, columns.header(True))

    def test_output_file_overrides_dir_and_name(self):
        override = os.path.join(tempfile.mkdtemp(), 'custom.csv')
        self.new_node(output_file=override)
        self.assertEqual(self.csv_path, override)
        self.assertTrue(os.path.isfile(override))

    def test_missing_output_configuration_raises(self):
        self._close()
        Stub.params = dict(Stub.params, output_dir='', output_file='',
                           output_file_name='')
        with self.assertRaises(ValueError):
            demonstrate.DemonstrateTemplateNode()

    # -- 表头与原有列 -----------------------------------------------------
    def test_header_keeps_original_columns_then_appends_extension(self):
        header, _ = self.read_csv()
        self.assertEqual(header, columns.header(True))
        self.assertEqual(header[:18], columns.HEADER_BASE)
        self.assertEqual(header[18:24], columns.HEADER_TCP)
        self.assertEqual(header[24:], columns.HEADER_EXT)

    def test_header_without_cartesian(self):
        self.new_node(record_cartesian=False)
        header, _ = self.read_csv()
        self.assertEqual(header, columns.header(False))
        self.assertEqual(len(header), 30)

    def test_normal_point_keeps_original_number_format(self):
        self.feed_sensors()
        self.feed_odom(1.25, -0.4, yaw_deg=45.0)
        self.assertEqual(self.node.write_record(False), 1)

        header, rows = self.read_csv()
        record = dict(zip(header, rows[0]))
        self.assertEqual(record['项点编号'], '1')
        self.assertEqual(record['底盘x'], '1.250')
        self.assertEqual(record['底盘y'], '-0.400')
        self.assertEqual(record['底盘yaw角度'], '0.785')
        self.assertEqual(record['关节1'], '57.300')   # 弧度 × 57.3，3 位小数
        for name in columns.HEADER_BASE[10:17]:
            self.assertEqual(record[name], '', name)
        self.assertEqual(record['项点名称'], '')
        self.assertEqual(record['tcp_x_mm'], '100.000000')  # 控制器原始单位

    # -- 末端位姿 ---------------------------------------------------------
    def test_end_effector_pose_comes_directly_from_tf(self):
        Stub.transforms[('map', 'Link6')] = _transform(
            1.5, -0.25, 1.05, (30.0, -20.0, 45.0))
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)

        header, rows = self.read_csv()
        record = dict(zip(header, rows[0]))
        self.assertEqual(record['map_frame'], 'map')
        self.assertEqual(record['ee_frame'], 'tf:Link6')
        self.assertAlmostEqual(float(record['ee_x_m']), 1.5, places=6)
        self.assertAlmostEqual(float(record['ee_y_m']), -0.25, places=6)
        self.assertAlmostEqual(float(record['ee_z_m']), 1.05, places=6)
        self.assertAlmostEqual(float(record['ee_rx_rad']),
                               math.radians(30.0), places=6)
        self.assertAlmostEqual(float(record['ee_ry_rad']),
                               math.radians(-20.0), places=6)
        self.assertAlmostEqual(float(record['ee_rz_rad']),
                               math.radians(45.0), places=6)

    def test_end_effector_pose_empty_when_tf_missing(self):
        # 桩里没有 map <- Link6：留空，但不影响整行写入
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        record = dict(zip(header, rows[0]))
        for name in columns.EXT_FLOAT_COLUMNS[:6]:
            self.assertEqual(record[name], '', name)
        self.assertEqual(record['ee_frame'], '')
        self.assertEqual(record['底盘x'], '1.000')

    def test_end_effector_disabled_by_empty_frame(self):
        self.new_node(ee_frame='')
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        self.assertEqual(dict(zip(header, rows[0]))['ee_frame'], '')

    # -- 过渡点 -----------------------------------------------------------
    def test_transition_uses_negative_numbers_and_previous_base(self):
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.assertEqual(self.node.write_record(False), 1)
        self.feed_odom(5.0, 6.0)          # 底盘动了，过渡点也不记录
        self.assertEqual(self.node.write_record(True), -1)
        self.assertEqual(self.node.write_record(True), -2)

        header, rows = self.read_csv()
        self.assertEqual(len(rows), 3)
        transition = dict(zip(header, rows[1]))
        self.assertEqual(transition['项点编号'], '-1')
        self.assertEqual(transition['项点名称'], columns.TRANSITION_POINT_NAME)
        self.assertEqual(transition['底盘x'], '1.000')
        self.assertEqual(transition['底盘y'], '2.000')
        self.assertEqual(transition['image_file'], '')     # 过渡点不拍照
        self.assertEqual(dict(zip(header, rows[2]))['项点编号'], '-2')

    def test_transition_can_record_live_base_when_enabled(self):
        self.new_node(transition_record_base=True)
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        self.feed_odom(5.0, 6.0)
        self.node.write_record(True)
        header, rows = self.read_csv()
        record = dict(zip(header, rows[1]))
        self.assertEqual(record['底盘x'], '5.000')
        self.assertEqual(record['项点编号'], '-1')

    # -- 图像与光源 -------------------------------------------------------
    def test_image_is_saved_under_item_number(self):
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        self.assertEqual(dict(zip(header, rows[0]))['image_file'], '1.png')
        expected = os.path.join(
            os.path.dirname(self.csv_path), 'teach_images', '1.png')
        self.assertEqual(Stub.written_images[expected], b'fake-frame')

    def test_image_skipped_when_unavailable(self):
        self.feed_arm()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        self.assertEqual(dict(zip(header, rows[0]))['image_file'], '')
        self.assertEqual(Stub.written_images, {})

    def test_light_columns(self):
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        record = dict(zip(header, rows[0]))
        self.assertEqual(record['light_white_brightness'], '128')
        self.assertEqual(record['light_white_on'], 'true')
        # 只记录白光：其他通道不上报也不影响
        self.assertEqual(record['ee_frame'], '')

    def test_light_columns_empty_without_state(self):
        self.feed_arm()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        record = dict(zip(header, rows[0]))
        self.assertEqual(record['light_white_brightness'], '')
        self.assertEqual(record['light_white_on'], '')

    def test_light_brightness_zero_is_recorded(self):
        # 亮度 0 是合法值，不能被当成“没数据”丢掉
        self.feed_arm()
        self.send('/light/state', _light_state(white_on=False, white_rate=0))
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        record = dict(zip(header, rows[0]))
        self.assertEqual(record['light_white_brightness'], '0')
        self.assertEqual(record['light_white_on'], 'false')

    # -- 图像旋转角 -------------------------------------------------------
    def test_rotation_angle_known_values(self):
        # 光学系 +Z 为光轴：水平前视且图像正立对应 rx = -90°
        self.assertAlmostEqual(
            demonstrate.image_rotation_deg(_rotation_matrix(-90.0, 0.0, 0.0)),
            0.0, places=9)
        self.assertAlmostEqual(
            demonstrate.image_rotation_deg(_rotation_matrix(-90.0, -30.0, 0.0)),
            30.0, places=9)

    def test_rotation_angle_degenerate(self):
        self.assertIsNone(
            demonstrate.image_rotation_deg(_rotation_matrix(0.0, 0.0, 0.0)))
        self.assertIsNone(
            demonstrate.image_rotation_deg(_rotation_matrix(180.0, 0.0, 0.0)))

    def test_rotation_angle_from_camera_tf(self):
        self.new_node(camera_frame='camera_optical_frame')
        Stub.transforms[('map', 'camera_optical_frame')] = _transform(
            1.0, 2.0, 1.2, (-90.0, -30.0, 0.0))
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        self.assertAlmostEqual(
            float(dict(zip(header, rows[0]))['image_rotation_deg']),
            30.0, places=3)

    def test_rotation_angle_empty_without_camera_frame(self):
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        header, rows = self.read_csv()
        self.assertEqual(dict(zip(header, rows[0]))['image_rotation_deg'], '')

    # -- 兼容性 -----------------------------------------------------------
    def test_generated_csv_passes_patrol_compat_check(self):
        self.feed_sensors()
        self.feed_odom(1.0, 2.0)
        self.node.write_record(False)
        self.feed_odom(3.0, 4.0)
        self.node.write_record(True)
        self.node.write_record(False)
        self.node.file.close()

        del check_csv_compat.ERRORS[:]
        del check_csv_compat.WARNINGS[:]
        check_csv_compat.check_file(self.csv_path)
        self.assertEqual(check_csv_compat.ERRORS, [])

    def test_missing_robot_status_blocks_cartesian_row(self):
        # 保持原有行为：需要笛卡尔列但机械臂不可用时整行不写
        self.feed_odom(1.0, 2.0)
        with self.assertRaises(RuntimeError):
            self.node.write_record(False)
        _, rows = self.read_csv()
        self.assertEqual(rows, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
