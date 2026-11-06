# Luna 头部/左右腕部 AprilTag 手眼标定

独立 ROS 2 Humble 包，不修改 URDF、MJCF、相机驱动或控制器，不发布任何运动指令。
默认只读加载已有 `Gento_Luna.xml`，以 `base_link` 为基坐标系。
头部、左手和右手各采集一次、各保存一份外参；不需要同时看到标定板。

## 快速开始

以下命令都从工作区根目录执行。标定程序不会移动机器人；请先用现有控制方式把机器人
移动到不同、稳定且安全的姿态。

```bash
# 1. 加载环境并构建
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select marvin_description luna_handeye_calibration --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 2. 启动头部采集器（左/右手分别换用对应 params-file）
ros2 run luna_handeye_calibration collect_apriltag --ros-args \
  --params-file robot_node/luna_handeye_calibration/config/head_camera.yaml \
  -p output:=calibration_data/head_samples_session01.yaml

# 3. 每到一个停稳的新姿态采一组，至少 12 组，建议 20–40 组
ros2 service call /luna_apriltag_collector/capture std_srvs/srv/Trigger '{}'

# 4. 离线求解外参；输出文件必须使用新名称
ros2 run luna_handeye_calibration solve_handeye \
  calibration_data/head_samples_session01.yaml \
  --output calibration_data/head_extrinsic_session01.yaml \
  --child-frame luna_head_calibrated_optical

# 5. 验证通过后发布动态 TF
ros2 launch luna_handeye_calibration camera_tf.launch.py \
  calibration_file:=calibration_data/head_extrinsic_session01.yaml \
  joint_topic:=/tj/joint_states
```

左手和右手的完整命令见“手动采集”和“离线求解”章节。腕部驱动如果不发布
`CameraInfo`，必须在配置中提供与实际相机、分辨率和 optical frame 匹配的
`intrinsics_file`；不能直接套用头部内参。

## 阅读顺序与功能边界

第一次使用按“构建 → 话题检查 → 内参准备 → 采集 → 求解 → TF → 独立验证”执行。
先完成头部流程，再复用到左手和右手。以下命令默认从**工作区根目录**执行，
不是从本包目录执行；每个新终端都需要加载 ROS 和工作区环境。

| 功能 | 当前实现 |
| --- | --- |
| 标签检测 | 单个指定 ID 的 AprilTag 36h11，不是多标签板联合优化 |
| 相机内参 | 读取已有 CameraInfo 或内参 YAML；支持 RealSense inverse Brown-Conrady |
| 外参求解 | 每个相机独立 Eye-in-Hand；基坐标默认 base_link |
| 机器人位姿 | Pinocchio 读取现有模型，用实际关节位置计算 FK |
| 时间匹配 | 图像时间两侧检查关节历史，停稳后选最近反馈，不做运动中插值 |
| 头部主机时间流 | 可选接收时间重发，图像与同帧内参共用新时间戳；原始流不变 |
| 采样触发 | 终端/图像窗口按键，或手动调用 Trigger 服务；成功才保存 |
| 实时预览 | 标签框/ID、PnP 状态与坐标轴、图像年龄及采样响应 |
| 机器人控制 | 无；不会切模式、使能、回 Home 或自动移动关节 |
| 部署 | 独立 YAML + 动态/安装座静态 TF，不改原有机器人描述 |

程序不能代替机械安装检查、关节零位标定、时钟同步或安全运动控制。
相机必须随所选安装 link 刚性运动；固定在环境中的相机属于另一种建模。

## 文件结构

```text
luna_handeye_calibration/
├── CMakeLists.txt / package.xml
├── config/
│   ├── head_camera.yaml
│   ├── left_hand_camera.yaml
│   └── right_hand_camera.yaml
├── launch/
│   ├── calibration.launch.py
│   ├── head_host_time_calibration.launch.py
│   └── camera_tf.launch.py
├── luna_handeye_calibration/
│   ├── collect_apriltag.py       # 消息订阅、采样服务、数据落盘
│   ├── head_host_time.py         # 可选头部主机接收时间流
│   ├── capture_keyboard.py      # 真实终端单键采样
│   ├── capture_preview.py       # 实时图像 + 终端/窗口统一按键采样
│   ├── apriltag.py               # 标签检测、PnP 双解过滤
│   ├── intrinsics.py             # 采集/预览共用的内参检查
│   ├── kinematics.py             # 相对 base_link 的正向运动学
│   ├── sampling.py               # 时间匹配和停稳检查
│   ├── solve_handeye.py          # 初始化、优化、留出验证
│   ├── camera_tf_broadcaster.py  # 外参加载及 TF 发布
│   └── se3.py                    # 变换检查和原子 YAML 写入
└── test/                        # 合成数据及隔离 ROS 节点测试
```

## 坐标约定和前提

`T_A_B` 将 B 坐标中的点变换到 A；平移单位米，关节角单位弧度，四元数 xyzw。
每组数据满足 `G_i X C_i = Y`：

- `G_i = T_base_mount(q_i)`：实际关节反馈计算的 FK。
- `C_i = T_camera_optical_tag`：AprilTag 四角及已知内参计算的 PnP。
- `X = T_mount_camera_optical`：要标定的固定安装外参。
- `Y = T_base_tag`：采集期间固定的标定板位姿，不必事先测量。

**采集期间 base_link 与标签必须相对静止，确保 Y 固定。** 头部标定建议固定底盘和躯干，
只改变头部两个轴。以 base_link 为基座且 FK 包含腿部反馈时，躯干运动并非数学上禁止，
但没有必要用它增加头部采样的复杂度。相机必须刚性固定于所选 mount；
若腕部安装实际随其他活动关节运动，应调整 mount，而非用优化掩盖错误。

默认 mount：头部 `Head_J2_Link`，左/右手 `Arm_L7_Link`/`Arm_R7_Link`。
MJCF 和 Luna URDF 中均存在 `base_link`。传入缺少该 frame 的模型会明确报错，不会把模型根坐标
偷偷当成 base_link。以 base_link 为基座时，头部 FK 会包含 base 到躯干之间的下肢关节；
因此采集消息必须包含相关的 `Leg_J1_Joint` 到 `Leg_J6_Joint` 反馈。模型的头部第二关节为 `Head_J2_Joint`，当前硬件反馈
为 `Head_J2_joint`，头部配置显式提供此名称映射。全程使用反馈而非命令或 Home 目标角度。

本程序无需预先添加相机 frame，也不需要关节雅可比；求解使用
OpenCV `calibrateHandEye(PARK)` 初始化，加 SciPy 鲁棒非线性位姿优化。
注意：相机和安装 link 原点不同，二者几何雅可比一般不相等，不能直接照搬。

## 构建和依赖

需要 ROS Humble 的 rclpy、cv_bridge、tf2_ros、Pinocchio Python、NumPy、SciPy、
PyYAML 和带 `aruco` 的 OpenCV（支持 4.5 旧接口和 4.7+ 新接口）。不自动安装/更新系统依赖。
本机 ROS 依赖位于系统 Python，不使用 Conda Python：

```bash
export PATH=/usr/bin:$PATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select luna_handeye_calibration --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

上面假设 `marvin_description` 已构建并可由工作区找到。新工作区先构建该包，再构建本包：

```bash
colcon build --packages-select marvin_description --symlink-install
source install/setup.bash
```

如果激活过 Conda，建议先退出环境，再加载 ROS；仅修改 PATH 不会清除全部 Conda
库路径影响。`PYTHONNOUSERSITE=1` 隔离用户目录中与 ROS 不兼容的包，不是跳过测试。
可先做不连接机器人、不移动硬件的依赖检查：

```bash
/usr/bin/python3 - <<'PY'
import cv2, numpy, scipy, yaml, pinocchio, rclpy, cv_bridge
assert hasattr(cv2, 'aruco'), 'OpenCV 缺少 aruco 模块'
assert hasattr(cv2.aruco, 'DICT_APRILTAG_36h11'), '缺少 AprilTag 字典'
assert hasattr(cv2, 'calibrateHandEye'), '缺少手眼标定接口'
assert hasattr(pinocchio, 'buildModelFromMJCF'), 'Pinocchio 不支持读取 MJCF'
print('依赖检查通过，OpenCV:', cv2.__version__)
PY
ros2 pkg executables luna_handeye_calibration
```

应看到 `collect_apriltag`、`solve_handeye`、`camera_tf_broadcaster`、`head_host_time`、
`capture_keyboard`、`capture_preview` 六个入口。
缺依赖时按部署环境管理方式补齐，不要用 Conda/pip 直接替换正在运行的 ROS 图像库。

测试（不控制硬件；ROS 测试使用隔离 domain 173）：

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m pytest robot_node/luna_handeye_calibration/test -q
colcon test --packages-select luna_handeye_calibration --event-handlers console_direct+
colcon test-result --test-result-base build/luna_handeye_calibration --verbose
```

## 1. 准备标定板和内参

### 1.1 上机前只读检查

通过现有启动流程启动相机和机器人反馈，不要再打开同一个相机设备。
各终端的 `ROS_DOMAIN_ID`、DDS 配置和时钟须与机器人一致。以下名称以实际发布为准：

```bash
ros2 topic list -t
ros2 topic info /tj/joint_states -v
ros2 topic echo /tj/joint_states --once
ros2 topic info /camera/camera/color/image_raw -v
ros2 topic echo /camera/camera/color/image_raw --field header --once
ros2 topic echo /camera/camera/color/camera_info --once
```

检查 `JointState.name` 与 `position` 数量一致、关节名称匹配模型、时间戳不为零。
采样器采用 SensorDataQoS；只有话题名字不代表实际收到数据，必须有发布者。
如果实际关节话题为 `/joint_states`，修改 `joint_topic`，不要照搬命名空间。

| 相机 | 默认图像话题 | 默认安装 link | 内参来源 |
| --- | --- | --- | --- |
| 头部 | `/camera/camera/color/image_raw` | `Head_J2_Link` | 同前缀的 `camera_info` |
| 左腕 | `/wrist_left_camera/camera/color/image_raw` | `Arm_L7_Link` | 额外提供 CameraInfo 或内参文件 |
| 右腕 | `/wrist_right_camera/camera/color/image_raw` | `Arm_R7_Link` | 额外提供 CameraInfo 或内参文件 |

### 1.2 AprilTag 规格

使用 **AprilTag 36h11**，默认 ID 0。将标签贴在刚性平板上，精确测量最外侧黑色方框
的边长（不包含白色留边），例如 `0.08` 米。不要把打印纸尺寸作为标签尺寸。
让图案清晰、无反光，保留白边，避免过小或近正视导致的平面 PnP 双解。

打印时避免“适应纸张”导致未知缩放，贴好后再测量实际边长。
同一图像内不能出现两个相同的目标 ID，其他 ID 不参与联合求解。
标定板可以在三个相机的不同会话之间重新放置，但同一会话内部不能移动。

### 1.3 准备匹配的相机内参

使用原始单相机图像，不能使用 `quad_tile/compressed` 拼图；图像与内参必须来自同一设备、
相同分辨率、同一 optical frame。当前支持 `plumb_bob`/`rational_polynomial` 和
RealSense 的 `inverse_brown_conrady`；不支持鱼眼、未经折算的 ROI/binning 或将校正图像和原始畸变参数混用。
RealSense inverse 模型会按 SDK 的 distorted→undistorted 方向先校正 AprilTag 角点，
再用零畸变 PnP，不能把它直接改名成 `plumb_bob`。

头部目前使用 `/camera/camera/color/image_raw` 和 `/camera/camera/color/camera_info`。
**当前自定义腕部驱动只发布图像，不发布 CameraInfo**。两个手部配置中的 CameraInfo
话题是待提供的接口，不代表已存在：需要匹配的 CameraInfo 发布者，或者用 `intrinsics_file`
显式提供对应腕部相机的内参 YAML（可以由该设备实际视频 profile 的 SDK 内参导出）：

```yaml
image_width: 640
image_height: 480
optical_frame: 实际图像header.frame_id
distortion_model: plumb_bob
camera_matrix:
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_coefficients:
  data: [k1, k2, p1, p2, k3]
```

上述 fx 等是占位符，必须替换为真实数值，不能套用头部内参。文件优先于话题。
确认驱动写入的 frame_id 确实代表图像光学坐标，而不是仅有名字相似的机械坐标。
不要同时让两个驱动打开同一个 RealSense 设备。

当前两台 D405 的 SDK 内参已从 `rs-enumerate-devices --no-dds -c` 读取并保存到项目下：

```text
calibration_data/left_d405_intrinsics.yaml   # serial 260322270903
calibration_data/right_d405_intrinsics.yaml  # serial 260322275725
```

两个腕部配置已经分别引用对应文件。SDK 报告的彩色流畸变模型为
`inverse_brown_conrady`；采集器会按该模型先把 AprilTag 角点转换到无畸变平面，
再执行 PnP。文件中的分辨率、`optical_frame` 和序列号必须与实际相机保持一致；
若更换视频分辨率，应重新导出对应 profile 的内参，不能继续使用这两份 640×480 参数。

## 2. 手动采集，每个相机一个新文件

使用项目已有安全操作方式低速调整姿态，远离限位和碰撞区域；本包不负责移动机器人。
先关闭遥操作跟随，固定底盘/躯干和标定板。头部应组合改变两个轴，不要只点头或只摇头；
手臂使用多个旋转轴和不同观察角度。建议每个相机 20–40 组，至少 12 组。

在工作区根目录启动头部采集（根据实测话题修改 joint_topic）：

```bash
ros2 run luna_handeye_calibration collect_apriltag --ros-args \
  --params-file robot_node/luna_handeye_calibration/config/head_camera.yaml \
  -p output:=calibration_data/head_samples.yaml \
  -p tag_id:=0 -p tag_size:=0.08 -p joint_topic:=/tj/joint_states
```

每到一个姿态停稳至少 0.5 秒，在另一个已加载 ROS 环境的终端执行：

```bash
ros2 service call /luna_apriltag_collector/capture std_srvs/srv/Trigger '{}'
```

每次成功采样立即原子保存；Ctrl-C 正常结束。默认允许覆盖已有输出文件；如需保留旧文件，
请改用新的 `output` 路径。
不自动录制视频，只保存角点、内参、实际关节角、时间匹配误差、FK、PnP 和重投影误差。
图像时间前后都必须有足够近的关节反馈；停稳检查使用位置历史，不依赖头部速度字段。
出现“未覆盖图像时间”时等一帧后重试；持续失败应检查设备时钟，不要直接增大容差。

左/右手：将 params-file 改为 `left_hand_camera.yaml`/`right_hand_camera.yaml`，
output 改为不同路径；缺少 CameraInfo 话题时另加：

```text
-p intrinsics_file:=/你的路径/该腕部实际内参.yaml
```

也可启动 `ros2 launch luna_handeye_calibration calibration.launch.py output:=... params_file:=...`。

### 左右手完整采集命令

先将 `intrinsics_file` 换成真实文件路径，不能直接使用占位路径。
若已有正确 CameraInfo 话题，删除该参数即可。

左手：

```bash
ros2 run luna_handeye_calibration collect_apriltag --ros-args \
  --params-file robot_node/luna_handeye_calibration/config/left_hand_camera.yaml \
  -p output:=calibration_data/luna_left_session01.yaml \
  -p intrinsics_file:=/你的路径/left_intrinsics.yaml \
  -p tag_id:=0 -p tag_size:=0.08 -p joint_topic:=/tj/joint_states
```

右手：

```bash
ros2 run luna_handeye_calibration collect_apriltag --ros-args \
  --params-file robot_node/luna_handeye_calibration/config/right_hand_camera.yaml \
  -p output:=calibration_data/luna_right_session01.yaml \
  -p intrinsics_file:=/你的路径/right_intrinsics.yaml \
  -p tag_id:=0 -p tag_size:=0.08 -p joint_topic:=/tj/joint_states
```

建议顺序采集三个相机，上一台完成后 Ctrl-C，再启动下一台，继续使用相同的 capture
服务名。不要同时启动三个同名采集节点，否则服务响应来源不明确。
成功响应包含 `success: true`、样本编号和重投影误差；失败时查看 `message`，不会增加样本数。

### 键盘交互采集（无图像窗口）

先启动上面的采集器，或保持已有采集器运行；**不需要重启采集器**。
另开真实交互终端，从工作区根目录运行：

```bash
export PATH=/usr/bin:$PATH PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run luna_handeye_calibration capture_keyboard
```

聚焦该终端，按 `c` / `C` / 空格采集一组，**无需回车**；`h` / `?` 显示帮助，
`q` / `Ctrl-C` 退出按键终端。退出后采集器仍运行，已保存样本保留，不会自动求解。
不支持将该终端程序放在 `ros2 launch`、后台或管道中；退出/异常会恢复终端输入模式。

一次请求未完成时忽略其他采样按键，不排队、不自动重试。默认等待 5 秒，超时只表示
响应状态未知，**不表示保存失败**；继续等待且阻止新采样，先核对采集器日志/文件。
按键应短按，不要长按。改变姿态后停稳至少 0.5 秒再按，每次确认成功或具体拒绝原因。

### 实时图像与采样一体化（推荐，只运行一个前端）

图形桌面中，在另一个已加载环境的终端运行：

```bash
ros2 run luna_handeye_calibration capture_preview
```

也可以用统一的启动参数选择腕部相机；`ros2 launch` 的子进程通常没有交互式
TTY，因此在 launch 终端按 `c` 不会采样。需要终端键盘采样时，先用 launch
只启动采集器，再在第二个真实终端运行预览：

```bash
# 终端 1：选择 left_hand 或 right_hand
ros2 launch luna_handeye_calibration capture_preview.launch.py \
  camera:=left_hand preview:=false tag_id:=0 tag_size:=0.02

# 终端 2：在真实交互终端运行，可在终端或图像窗口按 c/空格
ros2 run luna_handeye_calibration capture_preview
```

右手将 `camera:=left_hand` 改成 `camera:=right_hand`。头部如果存在设备时间戳偏差，
终端 1 使用 `head_host_time_calibration.launch.py`（ID/尺寸参数相同），终端 2 命令不变。
`camera` 会选择对应的图像话题、CameraInfo/内参配置和默认采样文件；需要时仍可传入
`params_file:=...` 或 `output:=...` 覆盖默认值。需要覆盖旧采样文件时增加
`allow_overwrite:=true`。不要同时运行多个默认同名采集器。

窗口**自动只读查询正在运行的采集器参数**：实际图像/内参话题、内参文件、标签 ID/
尺寸、PnP 阈值及时间策略，不需要再手动填一套可能不一致的相机配置。
例如当前采集器使用主机时间流、ID 0、20 mm，窗口自动使用同样配置；切到腕部也复用
同一命令。不会重新打开相机设备，不启动采集器、不改变相机时间戳、不控制机器人。
未连接采集器或未读到配置时显示等待状态，不发送采样请求。

**一个 `capture_preview` 进程同时支持启动终端和图像窗口按键**，不用再运行
`capture_keyboard`。可以保持启动终端焦点输入 `c` / 空格采样，同时看图像窗口；
也可以聚焦图像窗口按相同按键。无需回车，`h` 显示帮助，`q` / `Ctrl-C` 结束合并前端；
点击窗口关闭按钮也会结束前端，但不会停止采集器或删除数据。

终端启用成功时启动日志显示“终端+图像窗口按键均已启用”；采样按键收到后会明确记录
“收到启动终端按键 C”或“收到图像窗口按键 C”，随后显示“正在采样”和保存/拒绝结果。
若完全没有按键日志，检查焦点、英文输入法和启动终端是否仍可交互。
在一个真实交互终端中运行本命令，不需要另开键盘前端；后台/launch/管道没有 TTY 时
仍支持图像窗口按键，并打印提示。终端 EOF/读失败只禁用终端输入，图像窗口继续运行，
不会无提示地退出；正常退出或异常时恢复原有终端模式。
两处按键共用同一采样客户端和未完成请求保护，不排队、不自动重试；应短按，避免长按。
这些按键不是全局快捷键，只作用于本程序窗口或启动它的终端。

| 显示 | 意义 |
| --- | --- |
| 灰色标签框及 ID | 检测到其他 ID，不参与当前标定 |
| 橙色目标框 | 目标可见，但缺少/不匹配内参、重复 ID 或 PnP 被拒绝 |
| 绿色目标框和坐标轴 | 本帧唯一目标的 PnP 通过原有质量阈值；轴 X 红、Y 绿、Z 蓝 |
| `IDs` / `age` / `Preview Hz` | 已检测 ID、图像时间戳年龄及实际检测刷新率 |
| `STALE / INVALID TIMESTAMP` | 图像停止更新、陈旧、零时间戳或时钟异常；旧画面不能当实时结果 |
| `Capture [terminal/window: C]` | 最近收到按键的来源与最新采样响应 |
| `saved` / `rejected` | 本前端成功/拒绝次数，不冒充会话累计数 |

OpenCV 默认字体不能绘制中文，窗口用简短英文，**完整中文原因和保存样本编号在终端**。
预览只检查标签、内参和 PnP，**绿色不代表可保存或外参已标定**：采集器仍执行时间同步、
关节停稳、FK、重复姿态等所有检查。按键调用采集器的最新图像采样，不保证就是窗口中
已经画出的那一帧；最终数据以采集器返回成功、写入的 YAML 为准。
不自动保存视频/截图，不修改或缩放用于检测的原始图像；拖动窗口大小只改变显示。
默认预览上限 12 Hz，订阅队列深度 1，只分析最新帧，避免为预览积压相机消息。

自定义命名空间时，服务及采集器节点须指向**同一个采集器**：

```bash
ros2 run luna_handeye_calibration capture_preview --ros-args \
  -p capture_service:=/head_collector/capture -p collector_node:=/head_collector \
  -p preview_fps:=12.0 -p request_timeout:=5.0
```

默认从 `/luna_apriltag_collector/capture` 推导节点名，只有改掉服务末尾 `capture` 时
必须另指定 `collector_node`。终端前端支持 `capture_service` 和 `request_timeout`。
窗口与采集器必须使用相同 DDS/domain；需带 GUI 的 OpenCV 和可用桌面/DISPLAY。
无桌面 SSH 使用终端前端，或配置经批准的图形转发；不要为显示窗口修改相机驱动。
如果采集器重启/切换相机，应重启预览重新读取配置；读取远端内参文件时，本机也须能
访问相同路径。关闭前端不关闭采集器；采满 20–40 组后单独结束采集器，再离线求解。

### 操作检查表

1. 用已有安全控制方式到达能完整看见标签的新姿态。
2. 停止运动，等待至少 0.5 秒，确认标签清晰可见。
3. 按 c/空格或调用 capture，确认服务返回成功。
4. 改变观察姿态后再采样，不要在一个姿态反复采集。
5. 保存 20–40 组后结束采集器，开始离线求解。

### 头部显式切换为主机接收时间流

如果头部原始时间戳存在大偏差，且明确选择与手部相同的主机到达时间策略，可使用
以下独立启动入口。先停止已有的采集器，不能启动两个同名 capture 服务；无需停止
相机驱动，也不能再打开相机设备：

```bash
ros2 launch luna_handeye_calibration head_host_time_calibration.launch.py \
  output:=calibration_data/head_samples.yaml tag_id:=0 tag_size:=0.02
```

`tag_size` 为实测黑色方框边长，米；该入口默认 20 mm，普通采集配置仍保持原来的默认值。
它同时启动 `head_host_time` 和头部采集器，基坐标仍为 `base_link`。

| 内容 | 原始输入（不改变） | 主机接收时间输出 |
| --- | --- | --- |
| 原始图像 | `/camera/camera/color/image_raw` | `/camera/camera/color/image_raw_host_time` |
| 相机内参 | `/camera/camera/color/camera_info` | `/camera/camera/color/camera_info_host_time` |

重发器先按原始 `header.stamp` 精确配对同帧图像与 CameraInfo，输出二者使用图像
ROS 接收回调记录的同一主机时间。保留原始像素、分辨率、光学 frame、K/D/R/P 等信息，
不使用固定的秒数偏移，也不在原始话题上添加第二个发布者。
订阅/发布队列深度为 1，缓存最多 4 帧，配对等待最多 0.2 秒；缺失内参、重复图像、
源时间回跳帧和分辨率/frame 不匹配的数据不会重发。源时间回跳会清空配对缓存。
采集器的 0.5 秒新鲜度、50 ms 两侧关节时间匹配及停稳检查仍然保留。

**这是一种明确选择的时间戳策略，不是对 RealSense 时钟换算根因的修复。**
手部在 SDK 收帧回调打主机时间，头部新流在 ROS 接收回调打主机时间，后者额外包含
驱动和 ROS/网络传输延迟；都不能被解释成精确曝光时刻或硬件同步。
队列深度 1 和配对超时不能检测上游已存在的所有图像积压，因此只用于连续、低延迟
视频流的停稳手眼采样，不用于运动中的精密视觉伺服。应在机器人与相机都停稳后再采样，
并用独立姿态检查外参精度。跨主机时仍需同步各主机系统时钟。

新采集文件和求解输出均记录 `timestamp_source: ros_host_arrival`，不得与原始时间策略
的旧会话混用。切换后用新输出路径，仍调用 `/luna_apriltag_collector/capture`。
其他图像消费者若需要这一策略，须显式订阅新话题；原始驱动、深度/metadata/TF 的时间戳
没有被改写，不能把新彩色流当作自动完成了 RGB-D 或 TF 时间同步。

### 项目内数据保存位置

从工作区根目录运行时，数据保存在项目下的 `calibration_data/`，不再使用 `/tmp`：

```text
calibration_data/                    # 已加入根 .gitignore，真实数据不提交到仓库
├── head_samples.yaml               # 当前头部默认采样文件
├── left_hand_samples.yaml          # 左手配置默认采样文件
├── right_hand_samples.yaml         # 右手配置默认采样文件
├── head_extrinsic.yaml              # 求解时显式指定的头部外参
├── head_extrinsic.yaml.report.yaml  # 求解器自动产生的验证报告
└── ros_logs/                       # 可选：运行时 ROS_LOG_DIR 指向此处
```

相对路径按**采集器进程的工作目录**解析；应先进入工作区根目录，不能从任意目录启动后
期待文件仍落到项目下。三个配置文件提供各自的默认 `output`；头部主机时间 launch
默认 `output=calibration_data/head_samples.yaml`，也可以显式覆盖。
预览只调用采样服务，真正保存的位置由采集器的 `output` 决定，不由预览决定。
运行中的采集器不支持动态改变路径，需要重新启动采集器；改路径本身不改时间同步、
停稳或标签质量阈值。只有采样成功才创建 YAML；失败不会写文件。

成功采样后查看真实配置和文件：

```bash
ros2 param get /luna_apriltag_collector output
ls -lh calibration_data/
```

保留样本、内参、验证报告和模型版本；数据虽位于项目内，也不要提交真实采集数据或
设备信息，仍应单独备份。示例中的 session01、默认样本/外参名称若已存在，必须换新名称，
不能覆盖原会话。求解器 `--output` 仍需显式填写，例如 `calibration_data/head_extrinsic.yaml`。
当前不支持对已有 YAML 继续追加；重启使用新会话文件，不会自动恢复或合并数据。

关键可配置阈值：`sync_tolerance=0.05` 秒，`max_age=0.5` 秒，
`stationary_window=0.3` 秒，`stationary_motion=0.003` 弧度（当前 Luna 全是转动关节），
`max_reprojection_px=1.5`，`ambiguity_ratio=1.5`。采集会拒绝重复姿态和不明确的 PnP 双解。

## 3. 离线求解和验收

```bash
ros2 run luna_handeye_calibration solve_handeye calibration_data/head_samples.yaml \
  --output calibration_data/head_extrinsic.yaml --child-frame luna_head_calibrated_optical
```

左/右手用自己的数据和输出，child-frame 分别取
`luna_left_hand_calibrated_optical`/`luna_right_hand_calibrated_optical`。

```bash
ros2 run luna_handeye_calibration solve_handeye calibration_data/luna_left_session01.yaml \
  --output calibration_data/luna_left_extrinsic01.yaml --child-frame luna_left_hand_calibrated_optical

ros2 run luna_handeye_calibration solve_handeye calibration_data/luna_right_session01.yaml \
  --output calibration_data/luna_right_extrinsic01.yaml --child-frame luna_right_hand_calibrated_optical
```

求解是离线操作，不需要继续连接机器人。这里使用普通命令行参数 `--output` 等，
与采集节点的 `--ros-args -p ...` 不同。base/mount/model 信息从采样文件读取，
当前求解器不接受旧草稿中的 `--mount-frame` 参数。

程序检测单轴/重复/低激励退化，固定留出每四组中的一组做独立验证（不参与拟合）。
输出旁边的 `.report.yaml` 包含训练/验证索引、各组标签位姿偏差、RMS 和最大误差。
默认训练与留出集均要求最大平移偏差不超过 2 cm、旋转不超过 3°；这是可配置的
一致性门槛，**不是保证绝对精度**。实际应按照抓取/定位要求收紧并用独立实测验证。
改变标签尺寸、内参或机器人运动学都可能造成系统误差，不能仅靠训练残差发现。

阈值参数为 `--translation-limit`（米）、`--rotation-limit-deg`。验证失败只写报告，
不产生可部署的外参；不要为“通过”而放宽门槛。标签移动、错误 FK、时间不同步需重新采集。

样本不足、格式错误、运动退化或优化未收敛时，可能在报告生成前退出，以终端错误为准。
外参文件或同名报告已存在时拒绝覆盖；重新求解使用新的输出名称。

头部只有两轴不意味着不能标定 6D 外参，但必须有非平行轴运动；实际可观性由数据决定。
建议额外采集未用于本次求解的姿态，验证变换后的标签在 base_link 下保持不变。

## 4. 运行时发布 TF（无需改模型）

```bash
ros2 launch luna_handeye_calibration camera_tf.launch.py \
  calibration_file:=calibration_data/head_extrinsic.yaml joint_topic:=/tj/joint_states
```

默认计算 `T_base_camera(q)=T_base_mount(q) X`，发布动态 TF 到独立的标定 optical frame。
它与图像原始 optical frame 代表同一物理坐标，但名字不同，避免重设相机驱动已有 TF 的父节点。
消费者应显式用该外参/标定 frame 解释相机点，不能期待原始 frame 自动连接。
模型哈希不匹配会拒绝启动；新鲜反馈缺失则不发布，使用者也必须检查 TF 时间戳。
若现有 TF 中 base_link 或安装 link 尚未连到整机树，该节点不会伪造它们到 world 的变换。

如果已有准确 `base_link -> mount` 动态 TF，可选择 `mode:=static_mount`，只发布
`mount -> 独立相机光学frame` 静态变换。两种模式不能针对同一 child 同时运行；
三个相机同时发布时使用独立节点名和 child。绝不能把随关节运动的 `base_link -> camera` 发布为静态。

### 三个相机同时部署

在三个分别加载环境的终端运行，显式区分节点名：

```bash
# 终端 A：头部
ros2 run luna_handeye_calibration camera_tf_broadcaster --ros-args \
  -r __node:=luna_head_camera_tf \
  -p calibration_file:=calibration_data/head_extrinsic.yaml -p joint_topic:=/tj/joint_states

# 终端 B：左手
ros2 run luna_handeye_calibration camera_tf_broadcaster --ros-args \
  -r __node:=luna_left_camera_tf \
  -p calibration_file:=calibration_data/luna_left_extrinsic01.yaml -p joint_topic:=/tj/joint_states

# 终端 C：右手
ros2 run luna_handeye_calibration camera_tf_broadcaster --ros-args \
  -r __node:=luna_right_camera_tf \
  -p calibration_file:=calibration_data/luna_right_extrinsic01.yaml -p joint_topic:=/tj/joint_states
```

另开终端只读查询：

```bash
ros2 run tf2_ros tf2_echo base_link luna_head_calibrated_optical
ros2 run tf2_ros tf2_echo base_link luna_left_hand_calibrated_optical
ros2 run tf2_ros tf2_echo base_link luna_right_hand_calibrated_optical
```

每个 `tf2_echo` 持续运行，结束后 Ctrl-C。相机安装外参 X 固定，但这些 base_link 到相机
变换随关节变化。没有新的关节反馈时，TF buffer 可能仍能查到旧结果，因此还要核对时间戳。

### 在视觉程序中使用结果

如果目标点已经用匹配的内参和深度反投影到彩色相机光学坐标，记为 `p_camera`，则：

```text
p_base = T_base_mount(q_at_image_time) · T_mount_camera · [p_camera; 1]
```

原始深度图的像素不一定与彩色图对齐；必须先处理 RGB/深度内部外参和对齐，不能把同一个
像素索引直接当成对应点。手眼标定本身不会修复深度尺度、RGB/深度对齐或时间同步。
比较头手相机关系时，也应使用同一时刻的变换：

```text
T_headCamera_leftCamera = inverse(T_base_headCamera) · T_base_leftCamera
```

## 5. 参数参考

### 5.1 采集器参数（`--ros-args -p name:=value`）

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `image_topic` | `/camera/camera/color/image_raw` | 原始单相机 Image |
| `camera_info_topic` | `/camera/camera/color/camera_info` | 对应 CameraInfo |
| `intrinsics_file` | 空 | 显式内参文件，优先于话题 |
| `timestamp_source` | `image_header` | 时间策略记录：上游原始 header，或 `ros_host_arrival`；本参数不修改消息 |
| `joint_topic` | `/tj/joint_states` | 实际 JointState 反馈 |
| `output` | 裸节点为空；配置为 `calibration_data/<相机>_samples.yaml` | 新会话输出文件；已有文件拒绝覆盖 |
| `model_path` | 空 | 空时查找 marvin_description 中的 Luna MJCF |
| `base_frame` | `base_link` | 标定公共基坐标，须存在于模型 |
| `mount_frame` | `Head_J2_Link` | 相机刚性安装 link |
| `joint_aliases` | 头部 J2 名称映射 | JSON 字符串，模型名 → 反馈名 |
| `tag_id` | `0` | 36h11 字典中的标签 ID |
| `tag_size` | `0.08` | 黑色方框实测边长，米 |
| `max_reprojection_px` | `1.5` | PnP 四角重投影 RMS 上限，像素 |
| `ambiguity_ratio` | `1.5` | PnP 第二解/第一解误差比门槛，需大于 1 |
| `sync_tolerance` | `0.05` | 图像两侧相邻关节反馈各允许的最大时间差，秒 |
| `max_age` | `0.5` | 图像允许的最大年龄，秒 |
| `stationary_window` | `0.3` | 图像周围关节历史窗口，秒 |
| `stationary_motion` | `0.003` | 相关关节位置峰峰值上限；Luna 为弧度 |

头部默认别名为 `{"Head_J2_Joint": "Head_J2_joint"}`，左右手配置为 `{}`。
本版启动时读取这些参数，不支持通过 `ros2 param set` 动态重建检测器/FK/订阅。
若改变相机、标签、模型或内参，应重启并使用新数据文件。

### 5.2 求解器参数（普通命令行参数）

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `samples` | 必填位置参数 | 采集 YAML |
| `--output` | 必填 | 外参 YAML |
| `--child-frame` | 必填 | 未被现有节点使用的标定相机 frame |
| `--translation-limit` | `0.02` | 训练和留出样本的最大标签位置一致性误差，米 |
| `--rotation-limit-deg` | `3.0` | 最大标签旋转一致性误差，度 |

### 5.3 TF 节点参数（`--ros-args -p name:=value`）

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `calibration_file` | 空，实际必填 | 通过验证的外参文件 |
| `joint_topic` | `/tj/joint_states` | 动态模式的反馈话题 |
| `model_path` | 空 | 空时使用标定记录中的路径；换机器可改路径，但内容哈希须一致 |
| `mode` | `dynamic` | `dynamic` 或 `static_mount` |
| `max_age` | `0.5` | 关节反馈最大年龄，秒 |

`calibration.launch.py` 只提供 `params_file` 和 `output` 两个 launch 参数；其余写在参数
YAML 或使用 `ros2 run`。`camera_tf.launch.py` 只提供 `calibration_file`、`joint_topic`、
`mode`；要覆盖 model_path/max_age，使用 `ros2 run`。
`head_host_time_calibration.launch.py` 提供 `params_file`、`output`、`tag_id`、`tag_size`，
显式覆盖头部图像/内参话题和时间策略。仅运行重发器可用 `ros2 run luna_handeye_calibration head_host_time`；
可配置 `image_input_topic`、`info_input_topic`、`image_output_topic`、`info_output_topic`、
`max_pair_delay` 和 `max_pending_frames`。不支持 `use_sim_time=true`，输入/输出必须不同。

## 6. 数据格式与结果解释

### 采集文件

顶层包含 schema_version、坐标约定、base/mount/camera frame、模型路径和 SHA-256、
关节别名、标签规格、相机内参以及时间策略 `timestamp_source`。`samples` 中每组包含：

| 字段 | 含义 |
| --- | --- |
| `timestamp` | 图像时间戳，秒 |
| `joint_time_error_s` | 被选中的最近关节反馈与图像的时间差 |
| `joints` | 按名称存储的实际关节位置 |
| `mount_pose` | 4×4 的 T_base_mount |
| `camera_tag` | 4×4 的 T_camera_optical_tag |
| `corners_px` | AprilTag 四角像素坐标 |
| `reprojection_px` | 本帧 PnP 重投影 RMS |

求解器使用已保存的 mount_pose，不会在求解时悄悄使用另一个模型重新生成 FK。
不得手工交换矩阵方向、关节顺序或将角度单位从弧度改为度。

### 外参文件

`T_mount_camera.translation` 是光学原点在安装 link 中的位置，
`T_mount_camera.quaternion` 是将光学坐标方向变换到安装 link 的旋转（xyzw）。
`parent` 是安装 link，不是 base_link；动态发布器再通过 FK 乘上 base_link 到 mount。
`camera_frame` 记录采集时的原始光学 frame，`child` 是部署时独立的标定 frame 名。
`validation_passed` 只表示软件的一致性门槛通过，不表示机械/视觉绝对精度已验收。
`timestamp_source` 延续采集文件的时间策略；旧会话没有该字段时按 `image_header` 处理。

### 验证报告

- `train_indices`、`validation_indices` 是从零开始的样本编号；每四组的第四组留出。
- `excitation_singular_values` 用于旋转激励/退化检查，不是参数置信区间。
- `training`、`validation` 内的误差列表按各自索引顺序对应样本。
- `T_base_tag` 是拟合得到的固定标签在 base_link 下的位姿。
- 米级字段乘 1000 才是毫米；`rotation_*_deg` 已是度。

留出数据未用于优化，但来自同一次采集，不能替代另一轮独立实测。
建议在工作距离、不同图像区域和未用过的头/腕姿态再次观察固定标签，比较变换后的
base_link 坐标，并结合已知长度/实测位置检查尺度和系统偏差。

## 7. 常见故障排查

| 现象/报错 | 检查与处理 |
| --- | --- |
| 找不到包或可执行文件 | 重新 source install/setup.bash；检查构建结果和 Python 环境 |
| Pinocchio / cv_bridge / aruco 导入失败 | 使用系统 Python；检查 ROS 依赖，避免混用 Conda 和系统二进制库 |
| 未收到图像或 CameraInfo | 检查真实发布者、DDS/domain、命名空间；腕部需要额外内参流或文件 |
| optical frame 不匹配 | 对比图像 header 与 CameraInfo/文件 optical_frame，不要随意改名掩盖坐标错误 |
| 分辨率不匹配 | 使用当前视频 profile 的内参，禁止用预览缩放尺寸冒充原始尺寸 |
| 图像时间戳过旧/无效 | 检查相机时钟和主机同步、发布积压、零时间戳及仿真时间设置 |
| 关节时间戳未覆盖图像 | 等待新图像后重试；持续失败检查时钟偏差和反馈频率 |
| 停稳历史不足/反馈缺口 | 等待更多连续反馈，不要在启动瞬间采样 |
| 关节尚未停稳 | 停止运动和跟随，检查反馈抖动；不要仅依据速度字段为零 |
| 与已有姿态过于接近 | 改变观察姿态，重复帧不能增加有效激励 |
| 未发现指定 ID | 检查标签 family/ID、清晰度、白边、尺寸和遮挡 |
| 图像窗口打不开 | 检查桌面/DISPLAY 和 OpenCV GUI 支持；无桌面使用 capture_keyboard |
| 图像窗口等待采集器 | 先启动 collect_apriltag；检查节点名、参数服务、DDS/domain |
| 预览绿色但采样失败 | 查看终端完整响应；PnP 不代替同步、停稳和姿态检查 |
| PnP 双解不明确 | 适当倾斜观察，增大图像中的标签面积，改善光照 |
| 重投影误差过大 | 检查角点质量、正确内参/畸变和图像是否已被校正或缩放 |
| 模型缺少 base_link | 默认用 Luna MJCF；不能直接换成缺少 base_link 的模型 |
| 缺少关节反馈 | 按消息 name 检查别名和大小写，不能按数组下标猜测 |
| 运动退化/角度跨度不足 | 使用两个非平行旋转轴的多姿态组合，且不越过安全运动范围 |
| 训练/留出验证不通过 | 检查标签是否移动、躯干是否动、时间同步、FK 和关节零位 |
| 输出或报告已存在 | 使用新会话/新输出名称；不覆盖旧结果 |
| 模型哈希不匹配 | 核对模型版本并重新验证/标定，不手改哈希绕过检查 |
| TF 能查到但位置陈旧 | 检查最新反馈和 TF 时间戳；节点不收到新反馈就不会更新 |
| 原始 camera frame 仍不连通 | 本包故意发布独立标定 frame；视觉消费者须显式使用该外参 |

## 8. 已验证内容与上机验收

实现阶段的软件测试覆盖合成头部双轴/机械臂外参恢复、
噪声数据、错误留出样本拒绝、单轴退化、FK/别名、AprilTag 检测/PnP、采样时间匹配、
重复/陈旧数据拒绝、TF 时间戳、离线 CLI 输出和防覆盖，以及主机时间流的两种到达顺序、
内参/像素保留、重复/回跳/过期帧拒绝、有限缓存与非法配置拒绝。
交互测试覆盖真实 PTY 单键输入/异常恢复、方向键过滤、请求并发/超时保护、服务成功/
拒绝/异常响应、通过真实 ROS 参数服务读取配置及触发采样、预览框/多目标/PnP 双解、
共享内参检查、缓存图像陈旧提示以及 GTK 窗口可见性兼容。
合并输入测试覆盖终端采样同时保持预览、终端/窗口共用未完成请求保护、按键来源反馈、
终端模式恢复，以及终端 EOF/错误不关闭图像前端。
测试不发送任何机器人运动指令。
colcon 汇总可能额外计入 CTest 包装测试，数量会比 pytest 多一项。

这些是软件和合成数据验证，尚不代表已完成你的真实相机标定。上机验收还需：

- 确认相机刚性安装 link、设备对应关系和内参正确。
- 确认实际关节反馈及 FK 与物理运动一致，尤其头部角度/关节零位。
- 确认整个会话中 base_link 与标签相对静止、时间戳对齐。
- 用新的姿态和独立实测检查精度，满足任务要求后才供抓取/控制程序使用。
- 保留可追溯样本、报告和模型版本，不把“报告通过”当作安全认证。

## 参考

[OpenCV calibrateHandEye 坐标约定](https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html)
明确区分 gripper2base、target2cam 和 cam2gripper；本包用合成真值测试验证了此方向。
本包测试只验证软件逻辑和合成运动，实际相机安装、时钟、关节零位和精度仍需上机确认。
