# demonstrate

## 作用

ROS1 机械臂示教记录节点，采集底盘位姿、关节角和可选的 TCP 位姿，并写入 CSV 文件。

## 运行

```bash
roslaunch demonstrate demonstrate.launch
```

主要参数：

- `record_cartesian`：是否记录 TCP 位姿，默认 `true`。
- `output_file`：CSV 输出文件，由 launch 参数指定。

## 接口

订阅：

- `/joy_teleop/joy` (`sensor_msgs/Joy`)：手柄按键。
- `/Odometry` (`nav_msgs/Odometry`)：底盘位姿来源。
- `/joint_states` (`sensor_msgs/JointState`)：机械臂关节角。
- `/dobot_v4_bringup/msg/ToolVectorActual`：TCP 位姿，可选。
- `/dobot_v4_bringup/msg/RobotStatus`：机械臂连接状态，可选。

服务：

- `trigger_record` (`std_srvs/Trigger`)：记录一组示教数据。

手柄中，`button[3]` 记录普通点，`button[1]` 记录过渡点，`button[2]` 关闭节点。
