#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 ROS PositiveKin 服务为 CSV 追加 Link6 位姿。

CSV 中的关节角列使用度。项点编号为 0 的行会被完全跳过，不写入输出
CSV，也不会调用 PositiveKin 或校验该行的关节角。
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

try:
    import rospy
    from dobot_v4_bringup.srv import PositiveKin
except ImportError:  # 允许在没有 ROS 的环境中做参数和解析检查
    rospy = None
    PositiveKin = None


ITEM_ID_COLUMN = "项点编号"
JOINT_COLUMNS = ["关节1", "关节2", "关节3", "关节4", "关节5", "关节6"]
TCP_COLUMNS = [
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "tcp_rx_deg",
    "tcp_ry_deg",
    "tcp_rz_deg",
]


class PositiveKinError(RuntimeError):
    """PositiveKin 调用或返回值错误。"""


def is_zero_item_id(raw_value: str) -> bool:
    """判断项点编号是否为 0。支持 0、0.0 和带空格的写法。"""
    try:
        return float((raw_value or "").strip()) == 0.0
    except ValueError:
        return False


def parse_pose_string(raw_pose: str) -> List[float]:
    """解析服务返回的 ``x,y,z,rx,ry,rz`` 字符串。"""
    text = (raw_pose or "").strip().strip("{}").strip()
    values = [item.strip() for item in text.split(",") if item.strip()]
    if len(values) != 6:
        raise PositiveKinError(
            f"PositiveKin 返回的 pose 不是 6 个数值: {raw_pose!r}"
        )

    try:
        pose = [float(value) for value in values]
    except ValueError as exc:
        raise PositiveKinError(
            f"PositiveKin 返回的 pose 包含非数值内容: {raw_pose!r}"
        ) from exc

    if not all(math.isfinite(value) for value in pose):
        raise PositiveKinError("PositiveKin 返回的 pose 包含 NaN 或无穷大")
    return pose


class RosPositiveKin:
    """调用 dobot_v4_bringup 提供的 PositiveKin ROS 服务。"""

    def __init__(self, service_name: str, timeout: float) -> None:
        self.service_name = service_name
        self.timeout = timeout
        self._proxy = None

    def __enter__(self) -> "RosPositiveKin":
        if rospy is None or PositiveKin is None:
            raise RuntimeError(
                "当前 Python 环境没有 ROS 或 dobot_v4_bringup"
            )

        try:
            rospy.wait_for_service(self.service_name, timeout=self.timeout)
        except rospy.ROSException as exc:
            raise RuntimeError(
                f"找不到 ROS 服务 {self.service_name}，请先启动 dobot_v4_bringup"
            ) from exc

        self._proxy = rospy.ServiceProxy(self.service_name, PositiveKin)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self._proxy = None

    def positive_kin(
        self,
        joints_deg: Sequence[float],
        user_index: str,
        tool_index: str,
    ) -> List[float]:
        if self._proxy is None:
            raise RuntimeError("PositiveKin ROS 服务尚未连接")
        if len(joints_deg) != 6:
            raise ValueError("PositiveKin 必须接收 6 个关节角")

        try:
            response = self._proxy(
                J1=joints_deg[0],
                J2=joints_deg[1],
                J3=joints_deg[2],
                J4=joints_deg[3],
                J5=joints_deg[4],
                J6=joints_deg[5],
                user=str(user_index),
                tool=str(tool_index),
            )
        except Exception as exc:
            raise PositiveKinError(
                f"调用 ROS 服务 {self.service_name} 失败"
            ) from exc

        if response.res != 0:
            raise PositiveKinError(
                f"PositiveKin 失败，ErrorID={response.res}"
            )

        return parse_pose_string(response.pose)


def _parse_joint_values(row: dict, csv_row_number: int) -> List[float]:
    values = []
    for column in JOINT_COLUMNS:
        raw_value = (row.get(column) or "").strip()
        if not raw_value:
            raise ValueError(f"第 {csv_row_number} 行缺少字段 {column}")
        try:
            value = float(raw_value)
        except ValueError as exc:
            raise ValueError(
                f"第 {csv_row_number} 行字段 {column} 不是数字: {raw_value!r}"
            ) from exc
        if not math.isfinite(value):
            raise ValueError(f"第 {csv_row_number} 行字段 {column} 不是有限数值")
        values.append(value)
    return values


def output_path_for(input_path: Path) -> Path:
    return input_path.with_name(input_path.name + "_with_tcp.csv")


def convert_csv(
    input_path: Path,
    output_path: Path,
    positive_kin: RosPositiveKin,
    user_index: str,
    tool_index: str,
) -> Tuple[int, int]:
    """处理 CSV，返回 (转换数量, 跳过的 0 点数量)。"""
    if not input_path.is_file():
        raise FileNotFoundError(f"找不到输入 CSV: {input_path}")
    if input_path.resolve() == output_path.resolve():
        raise ValueError("输出文件不能覆盖输入文件")

    with input_path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None:
            raise ValueError("CSV 没有表头")

        required_columns = [ITEM_ID_COLUMN] + JOINT_COLUMNS
        missing = [column for column in required_columns if column not in reader.fieldnames]
        if missing:
            raise ValueError("CSV 缺少字段: " + ", ".join(missing))

        original_fields = list(reader.fieldnames)
        output_fields = [
            field for field in original_fields if field not in TCP_COLUMNS
        ] + TCP_COLUMNS
        rows = list(reader)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    converted_count = 0
    skipped_count = 0

    with output_path.open("w", newline="", encoding="utf-8-sig") as target:
        writer = csv.DictWriter(
            target,
            fieldnames=output_fields,
            extrasaction="ignore",
            lineterminator="\n",
        )
        writer.writeheader()

        for csv_row_number, row in enumerate(rows, start=2):
            output_row = {
                field: row.get(field, "")
                for field in output_fields
                if field not in TCP_COLUMNS
            }

            if is_zero_item_id(row.get(ITEM_ID_COLUMN, "")):
                # 0 点既不调用正解，也不写入输出 CSV。
                skipped_count += 1
                continue

            joints = _parse_joint_values(row, csv_row_number)
            pose = positive_kin.positive_kin(joints, user_index, tool_index)
            output_row.update(
                {
                    field: f"{value:.6f}"
                    for field, value in zip(TCP_COLUMNS, pose)
                }
            )
            writer.writerow(output_row)
            converted_count += 1

    return converted_count, skipped_count


def parse_args(argv: Iterable[str] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="通过 Dobot PositiveKin 为 CSV 追加 Link6 末端位姿"
    )
    parser.add_argument(
        "--csv",
        dest="input_path",
        default="/home/shansu/moma_github_ws/src/moma-patrol-system/demonstrate/path/402_1026_patroltest.csv",
        help="输入 CSV 路径",
    )
    parser.add_argument(
        "--output",
        help="输出 CSV 路径，默认在输入文件名后追加 _with_tcp.csv",
    )
    parser.add_argument(
        "--service",
        default="/dobot_v4_bringup/srv/PositiveKin",
        help="PositiveKin ROS 服务名",
    )
    parser.add_argument("--timeout", type=float, default=5.0, help="等待服务超时时间，单位秒")
    parser.add_argument("--user", default="", help="PositiveKin 的 User 编号，空字符串表示当前 User")
    parser.add_argument("--tool", default="", help="PositiveKin 的 Tool 编号，空字符串表示当前 Tool")
    return parser.parse_args(argv)


def main(argv: Iterable[str] = None) -> int:
    args = parse_args(argv)
    if rospy is None:
        print("转换失败: 当前环境没有 ROS Python 环境", file=sys.stderr)
        return 1

    rospy.init_node("hxd1d_positivekin_to_link6", anonymous=True)
    input_path = Path(args.input_path).expanduser()
    output_path = (
        Path(args.output).expanduser()
        if args.output
        else output_path_for(input_path)
    )

    print(f"输入 CSV: {input_path}")
    print(f"输出 CSV: {output_path}")
    print(f"PositiveKin 服务: {args.service}")
    print(f"User: {args.user or '当前 User'}，Tool: {args.tool or '当前 Tool'}")
    print(f"跳过项点编号为 0 的行: {ITEM_ID_COLUMN}=0")

    try:
        with RosPositiveKin(args.service, args.timeout) as positive_kin:
            converted_count, skipped_count = convert_csv(
                input_path,
                output_path,
                positive_kin,
                args.user,
                args.tool,
            )
    except Exception as exc:
        print(f"转换失败: {exc}", file=sys.stderr)
        return 1

    print(f"转换完成：正解 {converted_count} 行，跳过 0 点 {skipped_count} 行")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
