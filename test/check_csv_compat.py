#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""校验示教 CSV 是否仍能被 patrol 正常解析，并检查新增列的自洽性。

用法：
    python3 test/check_csv_compat.py <csv路径>
    python3 test/check_csv_compat.py --self-test

``--self-test`` 会用与节点相同的列定义生成两份样例（带/不带 tcp_* 列）
并对其自校验，可在没有 ROS 的机器上运行。
"""

import csv
import math
import os
import sys
import tempfile

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'demonstrate'))

from columns import (  # noqa: E402  pylint: disable=wrong-import-position
    HEADER_BASE,
    EXT_FLOAT_COLUMNS,
    TRANSITION_POINT_NAME,
    header as build_header,
)

TCP_COLUMNS = ['tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
               'tcp_rx_deg', 'tcp_ry_deg', 'tcp_rz_deg']
# patrol 会对这些列做 float() 且要求有限值
PATROL_FLOAT_COLUMNS = HEADER_BASE[1:10]
IMAGE_SUFFIXES = ('.png', '.jpg', '.jpeg', '.bmp')

ERRORS = []
WARNINGS = []


def fail(message):
    ERRORS.append(message)


def warn(message):
    WARNINGS.append(message)


def parse_float(text, where):
    try:
        value = float(text)
    except (TypeError, ValueError):
        fail(f'{where}: 不是有效数字 {text!r}')
        return None
    if not math.isfinite(value):
        fail(f'{where}: 不是有限数 {text!r}')
        return None
    return value


def read_rows(path):
    with open(path, 'r', newline='', encoding='utf-8-sig') as handle:
        reader = csv.reader(handle)
        header = next(reader, [])
        rows = [row for row in reader if row]
    return header, rows


def check_header(header):
    """返回是否为笛卡尔格式；同时校验列名与顺序。"""
    if not header:
        fail('表头为空')
        return False
    if header[:len(HEADER_BASE)] != HEADER_BASE:
        fail('前 %d 列与原有格式不一致: %s'
             % (len(HEADER_BASE), header[:len(HEADER_BASE)]))
    cartesian = 'tcp_x_mm' in header
    if cartesian:
        for name in TCP_COLUMNS:
            count = header.count(name)
            if count != 1:
                fail('笛卡尔模式下 %s 必须恰好出现一次，实际 %d 次' % (name, count))
    if header != build_header(cartesian):
        fail('表头与期望布局不一致：新增列必须追加在最后，原有列顺序不得变化')
    return cartesian


def check_row(index, row, header, cartesian):
    where = '第 %d 行' % index
    # patrol 要求行长度至少 18，否则整份文件加载失败
    if len(row) < len(HEADER_BASE):
        fail('%s: 只有 %d 列，patrol 要求至少 %d 列'
             % (where, len(row), len(HEADER_BASE)))
        return None
    if len(row) != len(header):
        fail('%s: 列数 %d 与表头 %d 不一致' % (where, len(row), len(header)))
        return None

    record = dict(zip(header, row))
    for name in PATROL_FLOAT_COLUMNS:
        parse_float(record[name], '%s %s' % (where, name))

    yaw = parse_float(record['底盘yaw角度'], '%s 底盘yaw角度' % where)
    if yaw is not None and not -math.pi - 1e-6 <= yaw <= math.pi + 1e-6:
        fail('%s: 底盘yaw角度 %s 超出 ±π（patrol 按弧度使用）' % (where, yaw))

    if cartesian:
        for name in TCP_COLUMNS:
            parse_float(record[name], '%s %s' % (where, name))

    for name in EXT_FLOAT_COLUMNS:
        if record[name].strip():
            parse_float(record[name], '%s %s' % (where, name))

    if record['light_white_brightness'].strip():
        brightness = parse_float(
            record['light_white_brightness'], '%s light_white_brightness' % where)
        if brightness is not None and not 0 <= brightness <= 255:
            fail('%s: light_white_brightness %s 不在 0..255 之间'
                 % (where, brightness))
        elif brightness is not None and brightness != int(brightness):
            fail('%s: light_white_brightness 应为整数，实际 %s' % (where, brightness))
    if (record['light_white_on'].strip()
            and record['light_white_on'] not in ('true', 'false')):
        fail('%s: light_white_on 只能是 true/false，实际 %r'
             % (where, record['light_white_on']))
    if not record['map_frame'].strip():
        fail('%s: map_frame 不能为空' % where)

    image_file = record['image_file'].strip()
    if image_file and not image_file.lower().endswith(IMAGE_SUFFIXES):
        fail('%s: image_file 扩展名不在 %s 中，实际 %r'
             % (where, IMAGE_SUFFIXES, image_file))

    if not record['检测类型'].strip():
        warn('%s 项点编号=%s: 检测类型为空，patrol 会把该项点记为异常'
             % (where, record['项点编号']))

    number = parse_float(record['项点编号'], '%s 项点编号' % where)
    if number is None:
        return None
    return number, record['项点名称'].strip()


def check_numbers(entries):
    """正式项点为正整数且互不相同，过渡点为负数。"""
    seen = set()
    for number, point_name in entries:
        is_transition = point_name == TRANSITION_POINT_NAME
        if number in seen:
            fail('项点编号 %s 重复' % number)
        seen.add(number)
        if is_transition:
            if number >= 0:
                fail('过渡点编号应为负数，实际 %s' % number)
        else:
            if number <= 0:
                fail('正式项点编号应为正整数，实际 %s' % number)
            elif number != int(number):
                fail('正式项点编号应为整数，实际 %s' % number)


def check_file(path):
    header, rows = read_rows(path)
    cartesian = check_header(header)
    if not header:
        return
    entries = []
    for index, row in enumerate(rows, start=2):
        entry = check_row(index, row, header, cartesian)
        if entry is not None:
            entries.append(entry)
    check_numbers(entries)

    point_count = sum(1 for _, name in entries if name != TRANSITION_POINT_NAME)
    transition_count = len(entries) - point_count
    print('文件: %s' % path)
    print('  列数: %d（笛卡尔列: %s）' % (len(header), '有' if cartesian else '无'))
    print('  数据行: %d' % len(rows))
    print('  正式项点: %d' % point_count)
    print('  过渡点: %d' % transition_count)


def make_sample(cartesian, path):
    """按节点写行的方式生成样例，用于自检。"""
    header = build_header(cartesian)
    normal = [
        '1', '1.250', '-0.400', '0.785',
        '10.000', '20.000', '30.000', '40.000', '50.000', '60.000',
        '', '', '', '', '', '', '', '',           # 7 个业务列 + 项点名称（空）
        '0.000000', '0.000000', '0.100000',
        '0.000000', '0.000000', '0.000000',        # tcp_* 六列
        'map', 'ToolVectorActual',
        '1.250000', '-0.400000', '1.235000',
        '-1.570796', '0.000000', '0.785398',
        '1.png', '12.500', '128', 'true',
    ]
    transition = [
        '-1', '1.250', '-0.400', '0.785',
        '20.000', '20.000', '30.000', '40.000', '50.000', '60.000',
        '', '', '', '', '', '', '', TRANSITION_POINT_NAME,
        '0.000000', '0.000000', '0.100000',
        '0.000000', '0.000000', '0.000000',
        'map', 'ToolVectorActual',
        '1.250000', '-0.400000', '1.235000',
        '-1.570796', '0.000000', '0.785398',
        '', '', '128', 'true',
    ]
    rows = [normal, transition]
    if not cartesian:
        # 去掉每行的 tcp_* 六列（索引 18..23）
        rows = [row[:18] + row[24:] for row in rows]
    for row in rows:
        assert len(row) == len(header), (len(row), len(header))
    with open(path, 'w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def self_test():
    exit_code = 0
    for cartesian in (True, False):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, 'sample.csv')
            make_sample(cartesian, path)
            del ERRORS[:]
            del WARNINGS[:]
            check_file(path)
            print('自检 cartesian=%s: %s'
                  % (cartesian, '通过' if not ERRORS else '失败'))
            for error in ERRORS:
                print('  [错误] %s' % error)
                exit_code = 1
            for warning in WARNINGS:
                print('  [提示] %s' % warning)
    return exit_code


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    if argv[1] == '--self-test':
        return self_test()

    check_file(argv[1])
    for warning in WARNINGS:
        print('[提示] %s' % warning)
    for error in ERRORS:
        print('[错误] %s' % error)
    print('结果: ' + ('通过' if not ERRORS else '%d 处错误' % len(ERRORS)))
    return 0 if not ERRORS else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
