#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""示教 CSV 的列定义，单独成模块以便节点与离线校验脚本共用。

改动约定
--------
* ``HEADER_BASE`` 是原有 18 列，**顺序与名称不得修改**，
  patrol 等下游按列号解析。
* ``HEADER_TCP`` 是原有可选 6 列，只在 ``record_cartesian`` 为真时出现，
  位置固定在 ``HEADER_BASE`` 之后。
* ``HEADER_EXT`` 是新增列，一律追加在最后，保证前 18(+6) 列不变。
"""

HEADER_BASE = [
    '项点编号', '底盘x', '底盘y', '底盘yaw角度',
    '关节1', '关节2', '关节3', '关节4', '关节5', '关节6',
    '检测类型', '项点类型', '报活处理班组', '架次',
    '方位', '左右位', '位别', '项点名称',
]

HEADER_TCP = [
    'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
    'tcp_rx_deg', 'tcp_ry_deg', 'tcp_rz_deg',
]

HEADER_EXT = [
    'map_frame', 'ee_frame',
    'ee_x_m', 'ee_y_m', 'ee_z_m',
    'ee_rx_rad', 'ee_ry_rad', 'ee_rz_rad',
    'image_file', 'image_rotation_deg',
    'light_white_brightness', 'light_white_on',
]

# 新增列中的数值列，供校验脚本判断是否需要按浮点数解析。
EXT_FLOAT_COLUMNS = [
    'ee_x_m', 'ee_y_m', 'ee_z_m',
    'ee_rx_rad', 'ee_ry_rad', 'ee_rz_rad', 'image_rotation_deg',
]

# 过渡点（项点名称为 TEMP）的底盘列沿用上一个正式项点。
TRANSITION_POINT_NAME = 'TEMP'


def header(record_cartesian):
    """返回完整的表头列表。"""
    return (
        list(HEADER_BASE)
        + (list(HEADER_TCP) if record_cartesian else [])
        + list(HEADER_EXT)
    )
