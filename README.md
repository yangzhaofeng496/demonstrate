# demonstrate

## 作用

ROS 1 机械臂示教记录节点。人在现场用手柄把底盘开到位置、把机械臂摆到位，按键把当前几何状态写进 CSV；该 CSV 由 `patrol` 读取执行巡检。

自本分支起，在**保持原有列完全不变**的前提下追加 12 列：末端在 `map` 下的位姿、每个正式项点的照片、光源亮度、以及让图像“重力朝下”的旋转角。

## 运行

```bash
roslaunch demonstrate demonstrate.launch
```

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `record_cartesian` | `true` | 是否记录原有 `tcp_*` 六列 |
| `config` | `config/demonstrate.yaml` | 输出路径、新增列、图像、光源、过渡点参数 |
| `output_file` | 空 | 整路径覆盖，通常留空；填了会忽略配置里的目录与文件名 |

输出位置写在配置文件里，与 `patrol.yaml` 的写法一致：

```yaml
output_dir: '$(find demonstrate)/path/'    # $(find) 由 launch 的 subst_value 展开
output_file_name: HXD1C_test.csv
output_file: ''                             # 整路径覆盖，一般留空
image_dir: ''                               # 空表示 <CSV 目录>/<CSV 名>_images/
```

**启动时以覆盖方式打开**该 CSV：同名旧示教文件会被清空。目录不存在会自动创建；`output_file_name` 未配置时节点直接报错，不会静默写到别处。

## 数据来源与转换链

原则：**优先使用已经融合好的原始数据，不做多余的坐标/单位换算。**

| CSV 列 | 最原始数据 | 到达方式 | 转换 | 步数 |
| --- | --- | --- | --- | --- |
| `底盘x/y/yaw角度` | 定位系统的 TF `map → base_link` | 订阅 `/Odometry` 只作采样节拍，值来自 TF | 四元数 → yaw | 1 |
| `关节1..6` | `/joint_states.position`（驱动原始弧度） | 订阅 | ×57.3 → 度 | 1 |
| `tcp_*`（原有） | 控制器 `ToolVectorActual`（毫米/度） | 订阅 | **无**，原样写入 | 0 |
| `ee_*`（新增） | TF `map → ee_frame`（定位 + URDF 臂链的融合结果） | `lookup_transform` | 四元数 → rpy | 1 |
| `image_file` | `/camera/image_raw` 图像帧 | 订阅 | `cv_bridge` 解码 → PNG 编码 | 1 |
| `image_rotation_deg` | TF `map → camera_frame` 的四元数 | `lookup_transform` | 四元数 → 3x3 → `atan2` | 2 |
| `light_brightness` / `light_on` | `/light/brightness_cmd`、`/light/power_cmd` | 订阅（latch） | **无** | 0 |

因为 TF 树本身已经融合了定位、里程计与 URDF，末端位姿与相机朝向各只需要**一次查询**；节点内没有任何外参拼接、毫米↔米或度↔弧度换算。位姿数学全部使用 ROS 自带实现（`tf2_ros.Buffer`、`tf.transformations`、`numpy`），不维护自定义几何库。

## 接口

订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/joy_teleop/joy` | `sensor_msgs/Joy` | 手柄按键 |
| `/Odometry` | `nav_msgs/Odometry` | 采样节拍 |
| `/joint_states` | `sensor_msgs/JointState` | 机械臂关节角（弧度） |
| `/dobot_v4_bringup/msg/ToolVectorActual` | `dobot_v4_bringup/ToolVectorActual` | 控制器 TCP（毫米/度） |
| `/dobot_v4_bringup/msg/RobotStatus` | `dobot_v4_bringup/RobotStatus` | 机械臂连接状态 |
| `/camera/image_raw` | `sensor_msgs/Image` | 末端相机图像 |
| `/light/brightness_cmd` | `std_msgs/Float32` | 光源亮度指令（只读） |
| `/light/power_cmd` | `std_msgs/Bool` | 光源开关指令（只读） |

服务：`trigger_record`（`std_srvs/Trigger`），与手柄等效。

手柄：`button[3]` 记录正式项点，`button[1]` 记录过渡点，`button[2]` 关闭节点；均为上升沿触发。

## CSV 列定义

前 18 列与可选 6 列是原有格式，名称、顺序、小数位均未改动；新增 12 列追加在最后。

| 段 | 列名 | 说明 |
| --- | --- | --- |
| 原有 18 列 | `项点编号 底盘x 底盘y 底盘yaw角度 关节1…关节6 检测类型 项点类型 报活处理班组 架次 方位 左右位 位别 项点名称` | 前 4 列 3 位小数；关节为“弧度 × 57.3”；后 7 列业务属性留空待人工填 |
| 原有可选 6 列 | `tcp_x_mm tcp_y_mm tcp_z_mm tcp_rx_deg tcp_ry_deg tcp_rz_deg` | 仅 `record_cartesian=true` 时出现，控制器原始单位，6 位小数 |
| 新增 12 列 | 见下 | 始终追加在最后 |

| 列名 | 单位 | 说明 |
| --- | --- | --- |
| `map_frame` | — | 位姿所在地图坐标系名（默认 `map`） |
| `ee_frame` | — | 实际使用的末端坐标系名，形如 `tf:Link6`；未取到时为空 |
| `ee_x_m` `ee_y_m` `ee_z_m` | m | 末端在地图系中的位置 |
| `ee_rx_rad` `ee_ry_rad` `ee_rz_rad` | rad | 末端姿态，固定轴 X-Y-Z 欧拉角 |
| `image_file` | — | 图片文件名，如 `1.png`；过渡点为空 |
| `image_rotation_deg` | ° | 见下文“图像旋转角” |
| `light_brightness` | 0–1 | 最后一次亮度指令；未收到时留空 |
| `light_on` | `true`/`false` | 最后一次开关指令 |

新增列取不到数据时留空并限频告警，**不影响整行写入**。原有列不降级：`record_cartesian=true` 而机械臂位姿不可用时，仍然整行不写（与旧版一致）。

## 几何与角度约定

- 位姿为 `[x, y, z, rx, ry, rz]`，旋转部分固定为**固定轴 X-Y-Z**：`R = Rz(rz)·Ry(ry)·Rx(rx)`，即 `tf.transformations.euler_from_quaternion(q, axes='sxyz')` 的输出顺序，与 `patrol` 的 `pose_matrix` 一致。
- `ry` 在 ±90° 附近存在万向锁，此时 `ee_rz_rad` 无独立意义。
- `map` 遵循 REP-103：**+Z 向上**。
- `ee_frame` 必须指向**实际末端**坐标系；它由 URDF 决定，例如 `Link6`（法兰）或 `tool0`。它与原有 `tcp_*` 列的控制器 TCP 是**两个不同来源**：前者来自 TF，后者来自控制器反馈；若二者理应重合，请把 `ee_frame` 指向与控制器 TCP 对应的坐标系。

## 过渡点

- 编号为**负数**：`-1, -2, -3…`，与正式项点编号互不干扰；
- `项点名称` 固定为 `TEMP`；
- 底盘列**默认沿用上一个正式项点的值**（`transition_record_base: false`），`patrol` 据此算出距离为 0，自动跳过底盘运动；
- 无前序正式点时回退实时位姿并告警；实时与沿用值偏差超过 0.05 m / 3° 时限频告警；
- 过渡点不拍照。

## 图像旋转角

`image_rotation_deg` 是让图像“重力朝下”所需**绕相机光轴旋转相机**的角度。正值表示绕 `+Z` 光轴（指向场景）按右手定则旋转；从相机背后看向前方时呈顺时针。若要转正像素，取相反数。

相机系必须是光学约定：**+X 向右、+Y 向下（图像 v 轴）、+Z 为光轴**。取 TF 查询到的四元数经 `quaternion_matrix` 转成旋转矩阵 `R` 后：

```text
g_cam = Rᵀ · (0, 0, -1) = -R[2, :]        # 重力在相机系中的表示
θ_deg = degrees(atan2(-g_cam.x, g_cam.y))
```

由于只用到 `R` 的第三行，实现就是两行 numpy。

注意：光学系 +Z 是光轴，**“水平前视且图像正立”对应 `rx = -90°`，不是 0**。

光轴与重力夹角小于 1° 时（例如正对地面俯拍），重力在像平面内没有投影，角度无意义，该列留空。

相机朝向来自 `camera_frame` 指定的 TF 坐标系；未配置或 TF 中不存在时该列留空。

## 拍照

记录正式项点时把最新相机帧保存为 `<image_dir>/<项点编号>.<image_format>`：

- `image_dir` 留空时默认 `<CSV 同目录>/<CSV 文件名>_images/`；
- 触发时要求图像不超过 `image_max_age`（默认 0.5 s）；超时按 `require_image` 决定是告警后继续写行（默认）还是报错不写；
- 同名文件覆盖并告警（CSV 本身也是覆盖写，语义一致）；
- 缺少 `cv_bridge` 或 `cv2` 时节点仍可运行，只是不拍照。

不调用 `moma_service/CapSavePic`：该服务在本工作空间无实现，实际提供方在版本管理之外的相机包中。

## 光源亮度

只被动订阅 `teleop_joy_ctrl` 已有的 latch 话题，不改动灯光节点。记录的是**指令值**，不是串口回读值；`teleop_joy_ctrl` 只在用户操作后才发布，因此会话首次调节光源之前这两列为空。

## 离线校验与测试

不需要 ROS 即可运行（测试内置了 rospy/tf2_ros/cv_bridge 的桩件）：

```bash
python3 test/test_write_record.py                     # 写行 + 旋转角，17 个用例
python3 test/check_csv_compat.py --self-test
python3 test/check_csv_compat.py /path/to/teach.csv   # 校验现场生成的 CSV
```

`check_csv_compat.py` 复刻 `patrol.read_waypoints_from_csv` 的解析规则（行长度 ≥ 18、前 10 列有限浮点、笛卡尔模式下 `tcp_*` 列唯一等），并额外检查编号规则与新增列自洽性。

## 已知限制

- 启动即以覆盖方式打开 `output_dir/output_file_name` 指向的 CSV，同名旧示教文件会被清空。
- 关节角换算使用常数 `57.3`（非精确 `180/π`），与旧版一致。
- 底盘位姿用 `/Odometry` 的时间戳查 TF，失败时保留上一次的值且不阻塞写行；三路数据（TF、关节、TCP）并非同一时刻的严格快照，示教应在车停稳后操作。
- 关节角按 `msg.position` 顺序取前 6 个，不按 `msg.name` 匹配。
- `检测类型` 等业务列由人工填写；留空时 `patrol` 会把该项点记为异常。
- `ee_frame` 与 `camera_frame` 依赖部署环境的 URDF / 定位 TF；本仓库当前分支不含 URDF。
