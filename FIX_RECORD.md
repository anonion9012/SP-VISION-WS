# 开发记录

## 仓库简介

本仓库是基于 `TongjiSuperPower/sp_vision_25` 改造的 ROS2 离线自瞄练习项目。项目使用 ROS2 节点接收 rosbag 中的图像和云台四元数，经过装甲板检测、PnP 解算、坐标变换和 Tracker 跟踪后，将目标位置、姿态和调试状态发布到 ROS2 话题，最后使用 PlotJuggler 观察数据曲线。

仓库主要目录如下：

- `src/sp_vision`：原有自瞄算法和 ROS2 主节点；
- `src/autoaim_msgs`：`/imu/quaternion` 使用的 `Orienta` 消息定义；
- `bags`：本地 rosbag 数据，不作为代码依赖提交；
- `shots`：各里程碑的编译、检测和 PlotJuggler 截图；
- `FIX_RECORD.md`：本次 ROS2 改造、排查和验证记录。

## 项目说明

### 项目功能

项目将原有单机版自瞄主循环改造成 ROS2 节点，处理链路为：

```text
rosbag 图像 + 云台四元数
        ↓
ROS2 订阅与逐帧配对
        ↓
装甲板检测
        ↓
PnP 解算与相机/云台/世界坐标变换
        ↓
Tracker EKF 跟踪
        ↓
目标位置、姿态和状态调试话题
```

### 输入

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/image_raw` | `sensor_msgs/msg/Image` | 1280×1024 的 BGR8 图像 |
| `/imu/quaternion` | `autoaim_msgs/msg/Orienta` | 当前帧对应的云台姿态四元数 |

由于 `Orienta` 消息没有时间戳，本项目使用 rosbag 录制端提供的“图像与姿态同帧同序”约定进行强制匹配。图像的 `header.stamp` 只用于 tracker 的帧时间和输出调试消息的时间戳。

### 输出

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/debug/armor_img` | `sensor_msgs/msg/Image` | 绘制检测框、分类和坐标轴的 debug 图像 |
| `/debug/armor_point` | `geometry_msgs/msg/PointStamped` | PnP 解算得到的装甲板世界坐标 |
| `/debug/target_point` | `geometry_msgs/msg/PointStamped` | EKF 旋转中心的世界坐标 |
| `/debug/gimbal_yaw` | `geometry_msgs/msg/Vector3Stamped` | `vector.x/y/z` 为云台 yaw/pitch/roll，单位为度 |
| `/debug/target_yaw` | `geometry_msgs/msg/Vector3Stamped` | `vector.x` 为 EKF 目标 yaw，单位为度 |
| `/debug/tracker_state` | `std_msgs/msg/String` | 可读的 Tracker 状态字符串 |
| `/debug/tracker_state_code` | `geometry_msgs/msg/Vector3Stamped` | `vector.x` 为数值化 Tracker 状态 |
| `/debug/image_count` | `std_msgs/msg/UInt64` | 已处理图像计数 |
| `/debug/imu_count` | `std_msgs/msg/UInt64` | 已消费 IMU 计数 |

所有新增的 `Vector3Stamped` 和点消息都使用对应图像的 `header.stamp`。Tracker 状态编码为：`lost=0`、`detecting=1`、`tracking=2`、`temp_lost=3`、`switching=4`。

### 参数入口

默认配置为 `src/sp_vision/configs/standard3.yaml`，也可以通过 ROS2 参数指定：

```bash
ros2 run sp_vision standard_ros2 \
  --ros-args \
  -p config_path:=/path/to/standard3.yaml
```

配置文件包含敌我颜色、模型路径、相机内参、相机到云台外参、云台到 IMU 外参、Tracker 参数和 `ros_imu_force_match` 等参数。

### 复现方法

```bash
source install/setup.bash
ros2 run sp_vision standard_ros2
```

另开终端回放数据：

```bash
source install/setup.bash
ros2 bag play bags/move_translate_bag --rate 0.3
```

再启动 PlotJuggler，观察以下曲线：

```text
/debug/armor_point/point/x
/debug/armor_point/point/y
/debug/armor_point/point/z
/debug/target_point/point/x
/debug/target_point/point/y
/debug/target_point/point/z
/debug/gimbal_yaw/vector/x
/debug/gimbal_yaw/vector/y
/debug/gimbal_yaw/vector/z
/debug/target_yaw/vector/x
/debug/tracker_state_code/vector/x
```

### 当前已知局限

1. 当前 bag 中姿态消息没有 header，只能使用录制顺序配对；更通用的方案是让姿态消息也携带和图像一致的时间戳。
2. 默认场景假设云台不平移，世界坐标只进行旋转补偿；如果云台或底盘发生平移，需要提供 `t_gimbal2world`。
3. 当前环境没有可用的 OpenVINO GPU，模型会回退 CPU，回放时应使用较低的 `--rate`，避免处理延迟过大。
4. debug 图像只有在存在订阅者时才发布，以减少无观察者时的图像编码开销。




## <div align = "center">M5</div>

### 实现
实现关键数据的发布，调整了主节点的运作逻辑。
![PlotJuggler关键数据绘制图](shots/M5-1.png)

图中左侧为 PlotJuggler 的 ROS2 话题树，右侧曲线包含目标位置、云台 yaw/pitch/roll、目标 yaw 和 Tracker 状态码。
状态码映射如下：
```cpp
if (state == "detecting") return 1;
if (state == "tracking") return 2;
if (state == "temp_lost") return 3;
if (state == "switching") return 4;
```

![ScreenShot](shots/录屏.webm)

---

### 修改

1. `standard_ros.cpp` 发布 `debug/armor_point` 和 `debug/target_point`，点坐标统一使用 `world` 作为 `frame_id`。
2. 新增带时间戳的 `debug/gimbal_yaw` 话题，消息类型为 `geometry_msgs/msg/Vector3Stamped`，`vector.x/y/z` 分别表示 yaw、pitch、roll，单位为度。
3. 新增带时间戳的 `debug/target_yaw` 话题，发布 Tracker EKF 旋转中心的 yaw，单位为度。
4. 新增带时间戳的 `debug/tracker_state_code` 话题，使用 `vector.x` 发布 Tracker 状态码，同时保留原有字符串话题 `debug/tracker_state`。
5. 修正 `Solver::world2pixel()`：将世界系到相机系的旋转矩阵先通过 `cv::Rodrigues()` 转成旋转向量，再交给 `cv::projectPoints()` 使用。
6. ROS2 IMU 强制匹配模式按 bag 约定按消息顺序消费姿态；IMU 超时不再复用上一帧姿态，而是丢弃当前未配对图像，避免错误姿态继续进入坐标变换。
7. 删除目标每帧坐标日志，并且只有存在订阅者时才发布 debug 图像，减少 PlotJuggler 和 CPU 回退环境下的处理阻塞。

### 验证

执行以下命令验证 ROS2 节点和调试话题：

```bash
colcon build --packages-select sp_vision --symlink-install
source install/setup.bash
ros2 run sp_vision standard_ros2
```

确认话题类型：

```bash
ros2 topic type /debug/gimbal_yaw
ros2 topic type /debug/target_yaw
ros2 topic type /debug/tracker_state_code
```

三个新话题的类型均为 `geometry_msgs/msg/Vector3Stamped`，时间戳使用当前图像消息的 `header.stamp`。

---

### 已知局限

- rosbag 中的 `Orienta` 消息没有自己的时间戳，只能按照录制端保证的图像/姿态同帧同序进行配对；如果上游发生丢帧或顺序改变，仍需要增加带时间戳的姿态消息。
- 当前默认云台不平移，因此世界坐标只补偿旋转，不包含 `t_gimbal2world`；如果以后加入底盘或云台平移，需要增加对应的位置输入。
- 当前机器没有可用的 OpenVINO GPU，模型会回退到 CPU，回放速度过快时仍可能出现处理延迟。
- `target_yaw` 发布的是 EKF 状态中的旋转中心角，不是每一块装甲板的瞬时朝向；EKF 没有独立的 target pitch/roll 状态，因此不发布伪造的 pitch/roll。

### 所遇问题与解决方案

#### 问题一：PlotJuggler 中目标曲线与云台运动不同步

初始 ROS2 实现使用两个独立订阅回调接收图像和四元数，并在图像回调中同步等待 IMU。图像检测和 CPU 推理耗时较长时，图像回调会阻塞，姿态消费和图像处理进度可能出现偏差。云台运动时，即使真实目标几乎不动，错误的姿态也会被带入 `R_gimbal2world`，最终表现为世界坐标抖动。

解决方案：利用 bag 中图像和四元数同帧同序的约定，在强制匹配模式下按消费顺序取姿态；IMU 不可用时直接丢弃当前未配对图像，不再沿用上一帧姿态。与此同时，删除每帧目标坐标日志，并降低回放速度，减少主回调被日志和模型推理拖慢的情况。

#### 问题二：世界坐标到图像的 debug 投影不正确

`world2pixel()` 原先将 3×3 旋转矩阵直接传给 OpenCV 的 `projectPoints()`。OpenCV 该接口要求的是 Rodrigues 旋转向量，因此 debug 图像中基于世界坐标的投影结果可能不正确。

解决方案：先使用 `cv::Rodrigues()` 将 `R_world2camera` 转换为 `rvec`，再调用 `cv::projectPoints()`。该修复只影响世界坐标的 debug 重投影，不改变 PnP 主流程。

#### 问题三：PlotJuggler 无法直接读取调试话题的时间戳

早期 yaw 和 Tracker 状态使用 `std_msgs/msg/Float64`，消息只有一个 `data` 字段，没有 `header.stamp`。PlotJuggler 只能使用接收时间，难以与图像、目标点和 rosbag 时间轴严格对齐。

解决方案：改用已有的 `geometry_msgs/msg/Vector3Stamped`，使用图像消息的 `header.stamp` 写入调试消息时间戳。云台姿态的 `vector.x/y/z` 分别发布 yaw、pitch、roll；目标 yaw 和 Tracker 状态码使用 `vector.x`。

#### 问题四：状态字符串不适合直接绘制离散曲线

`debug/tracker_state` 使用字符串消息，便于终端查看，但 PlotJuggler 不适合直接对字符串进行数值比较和绘图。

解决方案：保留字符串状态话题，同时新增 `debug/tracker_state_code`：`lost=0`、`detecting=1`、`tracking=2`、`temp_lost=3`、`switching=4`。这样既能查看可读状态，也能在曲线中观察状态切换。


## <div align = "center">M4</div>

### 实现
Tracker 稳定工作，PnP 解算修复，并在 ROS2 debug 图像中补充基于目标 EKF 状态的速度箭头。

---

### 修改
1. `standard_ros.cpp` 使用迭代器同步遍历装甲板和 Tracker 目标，避免两者错位访问。
2. 修正目标世界坐标状态索引：位置使用 `[x, y, z] = [x[0], x[2], x[4]]`。
3. 使用速度状态 `[vx, vy, vz] = [x[1], x[3], x[5]]` 预测 0.1 秒后的目标位置，并投影绘制速度箭头。

---

### 所遇问题与解决方案

#### 问题一：`standard_ros2` 运行后 tracker 始终处于 `lost`

`Tracker` 初始状态为 `lost`，只有检测到装甲板后才会进入 `detecting`，并在连续满足 `min_detect_count` 次检测后进入 `tracking`。如果 detector 返回的装甲板列表为空，`set_target()` 会持续返回 `false`，状态就会一直保持为 `lost`。

解决方案：排查 tracker 前先确认 detector 的输出数量、装甲板颜色、名称和类型，重点检查识别结果是否在进入 tracker 前已经为空。

#### 问题二：ROS2 图像回调同步等待 IMU，可能导致 tracker 被强制重置

`standard_ros.cpp` 在每次图像回调中同步调用 `ROSIMU::try_imu_at()`。该函数需要等待时间戳晚于当前图像的下一帧 IMU，超时时间为 100 ms。发生超时后虽然继续使用上一帧姿态，但图像处理节奏会产生阻塞和抖动。

同时，`tracker.cpp` 将两帧之间的时间间隔大于 100 ms 视为相机离线，并直接将状态设置为 `lost`，因此 IMU 等待或图像处理卡顿可能导致 tracker 反复丢失目标。

解决方案：避免在图像主回调中阻塞等待 IMU，使用最近一次有效姿态或独立线程完成姿态缓存；同时记录图像帧间隔和 IMU 超时次数，区分真实丢帧与同步等待造成的延迟。

#### 问题三：OpenVINO GPU 不可用时回退 CPU，可能造成推理帧间隔过大

配置文件中的 `device` 设置为 `GPU`，但当前环境没有可用的 OpenVINO GPU 设备，模型编译失败后回退到 CPU。CPU 推理耗时增加后，可能使图像回调间隔超过 tracker 的 100 ms 阈值，从而触发 `Large dt` 保护并进入 `lost`。

解决方案：运行时确认 OpenVINO 的 GPU 驱动和设备是否可用；如果使用 CPU，降低输入或模型推理负载，并在 tracker 中记录实际 `dt`，避免仅凭固定阈值误判相机离线。

#### 问题四：敌方颜色过滤可能清空 detector 输出

`tracker.cpp` 会根据配置中的 `enemy_color` 删除其他颜色的装甲板。当前 `standard3.yaml` 配置为 `enemy_color: "red"`；如果模型将目标识别为蓝色，或实际使用的机器人配置与该颜色不一致，所有检测结果都会在 tracker 中被过滤，最终表现为始终 `lost`。

解决方案：确认配置文件中的 `enemy_color` 与实际敌方颜色一致，并在 tracker 过滤前后分别输出装甲板数量和颜色，确认是否因为颜色过滤导致列表为空。

#### 问题五：ROS2 `config_path` 参数读取顺序错误

`standard_ros.cpp` 在构造函数体中才调用 `set_params()` 读取 ROS2 参数，但 detector、solver、tracker、aimer 和 shooter 都是在构造函数体执行前完成初始化的。因此通过 `--ros-args -p config_path:=...` 传入的配置路径不会被这些算法对象使用，它们仍然使用默认的 `standard3.yaml`。

解决方案：在构造所有依赖配置的算法对象之前完成参数解析，或将算法对象改为在参数读取之后显式构造，确保命令行指定的配置文件真正生效。

#### 问题六：debug 图像中的速度箭头使用了错误的目标状态

原实现将 EKF 状态向量的连续分量直接作为世界坐标，实际状态顺序为 `[x, vx, y, vy, z, vz, ...]`，因此会把速度分量误当作坐标，绘制出的线段也不能表示目标速度。

解决方案：按状态定义读取位置和速度，使用 `位置 + 速度 × 0.1 s` 得到预测点，再将当前位置和预测点分别投影到图像坐标后绘制箭头。

## <div align = "center">M3</div>

### 实现
本次实现装甲板检测与 debug 输出。

![3-1](shots/M3-1.png)

图中目标装甲板两侧灯条被蓝色边框识别，中间装甲区域被红色轮廓框出，并标注分类结果 `one`，说明传统检测链路已经输出了可供后续 PnP 使用的四点信息。

![3-2](shots/M3-2.png)

第二张截图展示了另一帧图像中的同一目标检测结果。目标位置发生变化后，检测框仍能覆盖装甲板，说明节点能够持续从图像回调进入 detector 和 debug 图像发布流程。

---

### 修改
1. `thread_safe_queue` 新增尝试读取方法。
2. `ros_imu` 修改 imu 数据获取，增加超时读取方法，避免永久阻塞。
3. `standard_ros` 识别器识别后新增 debug 话题。

---

### 已知问题
- [ ] debug 模块无法输出相应 Header。

--- 

### 所遇问题与解决方案

## <div align = "center">M2</div>

### 实现

本次目标是支持接收离线 bag 中的图像和四元数，并按时间戳及录制顺序匹配。

--- 

### 修改
1. `cboard.cpp` 使其支持从 ROS2 节点获取 imu 数据，并发布相应话题。
2. `standard3.yaml` 新增 `ros_imu_connect_can` 参数，用于控制在有 ros_imu 节点但无持续接受数据并传出时是否启用 CAN 接口模块代为接受数据。
3. `autoaim_msgs` 新增信息包。
4. `ros_imu.cpp` 新增 ROS2 节点接受 imu 数据，并按原有算法写入安全队列。
5. `path.hpp` 新增配置文件路径校准工具，并应用在 `yolov5.cpp`、`yolov8.cpp`、`yolov11.cpp`、`classifier.cpp`文件中。
6. `standard_ros.cpp` 修复了无法退出的问题。

---

### 已知问题

- [ ] 最终输出指令有误。

- [x] ~detached IMU 线程没有生命周期所有者。~

---

### 所遇问题与解决方案

#### 问题一：`ros_imu` 源码存在编译错误

启用 ROS2 `ros_imu` 编译后，暴露出 `create_subscription(...)` 语句缺少分号、缺少 `tools/logger.hpp`、回调参数使用 `ConstUniquePtr` 导致 unique_ptr 删除器类型不匹配等问题。

解决方案：补充分号和 logger 头文件，将订阅回调类型改为 `autoaim_msgs::msg::Orienta::ConstSharedPtr`，并让 ROS 回调直接把四元数数据写入线程安全队列。

#### 问题二：`ROSIMU` 构造阶段存在阻塞风险

原 `ROSIMU` 构造函数中启动了未定义的 `get_imu_data_thread()`，随后立即 `queue_.pop()` 两次等待 IMU 数据。ROS2 节点还未开始 spin 时构造函数就等待队列数据，存在构造阶段死锁风险。

解决方案：移除未实现的后台线程逻辑，改为在 `imu_at()` 首次调用时再从队列读取两帧初始数据用于插值。

#### 问题三：`ros2 run` 下模型资源路径依赖当前工作目录

source 工作区后再次运行节点，程序在加载分类模型时报错 `Can't read ONNX file: assets/tiny_resnet.onnx`。原因是配置文件中的模型路径为 `assets/...`，原代码直接按进程当前工作目录解析路径；通过 `ros2 run` 启动时当前目录不一定是 `src/sp_vision`，因此安装后的资源文件无法被稳定找到。

解决方案：新增路径解析工具 `tools/path.hpp`。当配置中的路径不是绝对路径且当前目录下不存在该文件时，按 `config_path` 所在包目录解析，例如将安装环境中的 `assets/tiny_resnet.onnx` 解析到 `install/sp_vision/share/sp_vision/assets/tiny_resnet.onnx`。分类器和 YOLOV5、YOLOV8、YOLO11 的模型加载均接入该路径解析逻辑。

#### 问题四：`standard_ros2` 默认配置路径仍依赖源码目录

`standard_ros.cpp` 原默认参数为 `src/sp_vision/configs/standard3.yaml`。从工作区根目录运行时该路径可能存在，但通过安装空间或其他目录运行时并不可靠；从 `src/sp_vision` 目录运行则会直接找不到配置文件。

解决方案：使用 `ament_index_cpp::get_package_share_directory("sp_vision")` 获取 ROS2 安装后的 package share 目录，将默认配置路径设置为 `share/sp_vision/configs/standard3.yaml`；同时在 `CMakeLists.txt` 和 `package.xml` 中补充 `ament_index_cpp` 依赖。

#### 问题五：OpenVINO 配置为 GPU，但当前机器没有可用 GPU 设备

修复模型路径后，节点继续在 YOLO 模型编译阶段崩溃。配置文件中 `device: GPU`，但当前环境没有 OpenVINO 可用的 GPU 设备，报错 `no supported devices found`。

解决方案：不切换 ONNX Runtime，保留原 OpenVINO 推理逻辑。在 YOLOV5、YOLOV8、YOLO11 的模型编译处增加最小 fallback：按配置设备编译失败且设备不是 CPU 时，记录警告并自动降级到 CPU 重新编译模型。模型、输入预处理、后处理逻辑保持不变。

#### 问题六：本机缺少 CAN 接口导致 SocketCAN 警告

节点成功启动后仍持续输出 `SocketCAN::open() failed: Error getting interface index!`。该问题来自当前机器没有配置文件中指定的 CAN 网络接口，属于硬件/系统网络接口环境问题，不是模型加载或 ROS2 节点构建错误。

解决方案：本次未改变 CAN 原有逻辑，仅确认该警告不再导致节点崩溃。实际机器人运行时需要确保配置中的 `can_interface` 对应系统中存在的 CAN 接口。

### 验证结果

执行以下命令完成验证：

```bash
colcon build --packages-up-to sp_vision --event-handlers console_direct+
source install/setup.bash && ros2 run sp_vision standard_ros2
```

验证结果：`autoaim_msgs` 和 `sp_vision` 均构建成功，`standard_ros2` 已成功编译、链接并安装。执行 `ros2 run sp_vision standard_ros2` 后，节点不再因 ONNX 路径或 OpenVINO GPU 设备不可用而崩溃，能够完成初始化并进入 ROS2 spin；剩余 SocketCAN 警告为本机缺少 CAN 接口导致。


## <div align = "center">M1</div>
### 实现
M1实现了自瞄系统的主入口ROS2节点化，可使用`colcon build`构建并能成功运行。
![构建示图](shots/colcon_build.png)

截图显示 `colcon build` 成功完成 `sp_vision` 包构建，并通过 `ros2 pkg list` 检查到已安装的 `sp_vision` 包，随后能够启动 `standard_ros2` 节点。
--- 

### 修改
1. 新增`standard_ros2`节点：ROS2化`standard.cpp`的主程序。
2. 修改`CMakeLists.txt`：删除了原有的依赖包检查和原有的ROS2构建步骤，修改为新的`standard_ros.cpp`构建步骤，并完善了包的安装。

--- 

### 已知问题

- [x] ~暂无测试程序对数据流程进行测试~

- [ ] 原有开源项目的代码全数保留，导致编译时间过长


## 第一次编译尝试与环境依赖解决
已将sp_vision整理成一个包，尝试构建，解决环境依赖问题。
### 所遇问题与解决方案
#### 问题一：首次构建命令被工具超时中断

执行 `colcon build --executor sequential` 后，命令在 1 秒前台执行限制内被中断，只输出了 `Starting >>> sp_vision`。

解决方案：使用更长的执行超时时间重新运行同一命令，确认实际构建结果，而不是将工具超时误判为编译失败。

验证结果：确认 `sp_vision` 随后确实进入编译阶段，并暴露出真实的依赖版本问题。

#### 问题二：链接阶段出现 fmt v8 未定义引用

构建过程中出现大量 `undefined reference to fmt::v8` 错误。项目源码和系统 `spdlog 1.9.2` 使用 `fmt 8.1.1` ABI，但 CMake 缓存中的 `fmt::fmt` 被解析到了 Anaconda 的 `/media/usr/Data/anaconda3/lib/libfmt.so.11.2.0`。

解决方案：重新配置构建时，将 `fmt_DIR` 和 `spdlog_DIR` 固定为系统 CMake 包目录，避免链接 Anaconda 的 `fmt 11`。

验证结果：链接命令改为使用 `/usr/lib/x86_64-linux-gnu/libfmt.so.8.1.1`，不再出现 `fmt::v8` 未定义引用。

#### 问题三：编译阶段混入 Anaconda 的 fmt v11 头文件

即使链接库已切换到系统版本，编译仍出现 `fmt::basic_runtime` 不存在以及 `spdlog` 格式字符串相关错误。检查生成的 `flags.make` 后发现编译参数包含 `-isystem /media/usr/Data/anaconda3/include`，导致 Anaconda 的 `fmt 11.2.0` 头文件覆盖了系统 `fmt 8.1.1` 头文件。

解决方案：继续检查 CMake 缓存和传递依赖，发现 Ceres 相关依赖仍使用 Anaconda 的 TBB、glog、gflags 配置，并将 Anaconda include 目录传递给目标。将 `TBB_DIR`、`gflags_DIR` 和 `glog_DIR` 也固定到系统 CMake 包目录。

验证结果：编译参数不再包含 Anaconda include；编译器使用系统 `fmt 8.1.1` 头文件，`spdlog` 相关错误消失。

#### 问题四：多个依赖同时被 Anaconda CMake 配置选中

构建缓存还曾使用 Anaconda 的 `yaml-cpp` 和 `nlohmann_json` 配置，造成头文件和库来源混杂，增加 ABI 不一致风险。

解决方案：将 `yaml-cpp_DIR` 固定为 `/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp`，将 `nlohmann_json_DIR` 固定为 `/usr/lib/cmake/nlohmann_json`，并与系统 `fmt`、`spdlog`、TBB、glog、gflags 配置一起重新配置构建。

验证结果：构建链接命令使用系统 `libyaml-cpp.so`、系统 `fmt`、系统 `glog`、系统 `gflags` 和系统 TBB，依赖来源统一。

#### 问题五：最终构建验证

修复后需要确认用户最初要求的原始命令可以直接运行，而不是只验证带额外 CMake 参数的命令。

解决方案：重新执行 `colcon build --executor sequential`。

验证结果：`sp_vision` 构建成功，耗时约 4.89 秒，最终摘要为 `1 package finished`。仅保留 ROS2 未找到和没有 `install` target 的警告，不影响编译结果。

#### 问题五：实际编译并行度为 20，导致系统高负载卡死

上次构建记录显示，虽然命令行使用了 `--parallel-workers 2` 和 `-DCMAKE_BUILD_PARALLEL_LEVEL=2`，但 Colcon 最终调用 Make 时仍执行了 `cmake --build ... -- -j20 -l20`。设备只有约 15 GiB 内存且没有交换分区，构建在 `auto_aim` 的 `yolo.cpp` 附近达到 39% 后中断，存在因编译任务过多导致桌面失去响应的风险。

解决方案：在工作区新增 `colcon_defaults.yaml`，将 Colcon 包并行度和 CMake 并行参数固定为 2；同时在用户 `.bashrc` 中设置 `MAKEFLAGS=-j2 -l2` 和 `CMAKE_BUILD_PARALLEL_LEVEL=2`，避免 Colcon 的 CMake 任务根据 CPU 核数自动追加 `-j20`。

#### 问题六：系统 spdlog 与 Anaconda 依赖混用

重新构建后发现，系统 `/usr/include/spdlog` 与 Anaconda 的 `fmt`、TBB、gflags、glog、yaml-cpp、nlohmann_json 配置混用，导致 `fmt::basic_runtime` 不存在等编译错误。该问题与之前记录的 fmt ABI 不一致问题属于同一类依赖来源污染。

解决方案：在 `colcon_defaults.yaml` 中固定使用系统 CMake 包目录：`fmt`、`spdlog`、`yaml-cpp`、`nlohmann_json`、`TBB`、`gflags` 和 `glog`，并重新配置 CMake 缓存，避免 Anaconda 的头文件和库继续混入构建。

#### 问题七：ROS2 目标缺少依赖传递和链接配置

启用 `standard_ros2` 后，首先出现 `cv_bridge/cv_bridge.h` 找不到，原因是目标没有通过 `ament_target_dependencies` 继承 ROS2 依赖的头文件目录。随后手动链接 `rclcpp`、`sensor_msgs` 和 `cv_bridge` 又导致链接器找不到对应的普通库名；移除这些手动库名后，还需要显式链接系统 `fmt::fmt`。

解决方案：在顶层 `CMakeLists.txt` 中为 `standard_ros2` 添加 `ament_target_dependencies(standard_ros2 rclcpp sensor_msgs cv_bridge)`，保留 `fmt::fmt` 等普通库的 `target_link_libraries` 配置，不再将 ROS2 包名当作普通库直接链接。

#### 问题八：新增 standard_ros.cpp 存在 ROS2 API 和 C++ 语法问题

`standard_ros.cpp` 中存在缺少类结尾分号、缺少程序入口、使用全局 `rclcpp::create_subscription`、错误读取参数，以及将 ROS 时间消息直接转换为 `std::chrono::nanoseconds` 等问题。

解决方案：改用节点成员函数创建订阅，分别使用 `sec` 和 `nanosec` 计算时间戳，改为通过节点成员读取参数，补充 `rclcpp::init`、节点创建、`spin`、`shutdown` 生命周期，并完善类定义结尾。

#### 问题九：ament_package 使用 Anaconda Python 导致 catkin_pkg 缺失

为 CMake 添加 `ament_package()` 后，构建进入 ROS2 包元数据处理阶段，但 CMake 使用了 `/media/usr/Data/anaconda3/bin/python3`，该环境中没有 ROS2 所需的 `catkin_pkg`，因此出现 `ModuleNotFoundError: No module named 'catkin_pkg'`，安装步骤无法完成。

解决方案：不额外安装 Python 依赖，避免继续混用 Anaconda 与 ROS2 系统环境。在工作区 `colcon_defaults.yaml` 的 CMake 参数中固定 `-DPython3_EXECUTABLE=/usr/bin/python3`，让 `ament_package()` 使用系统 Python 执行 `package.xml` 元数据转换脚本。
