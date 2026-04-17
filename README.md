# usb_camera

这是一个基于 ROS 2 的多源相机驱动节点，支持原生 V4L2 USB 摄像头、网络摄像头 (Web Camera) 以及通过 ZeroMQ (ZMQ) 接收纯内存字节流的图像透传。

该包特别优化了跨环境（如 Windows 宿主机到 WSL）的硬件直通问题。通过 ZMQ 接收原始图像字节流，绕过复杂的视频流编解码开销，实现低延迟的图像传输，非常适合工业相机（如海康相机）在虚拟机或 WSL 环境下的 ROS 2 接入。

## 特性

* **多数据源支持**：
    * **ZMQ 内存透传**：直接读取发送端 `tobytes()` 的原始内存数据，零编解码延迟。
    * **USB 相机 (V4L2)**：支持直接操作底层 V4L2 设置像素格式 (MJPEG, YUYV, NV12, RGB24) 以获取最高性能，随后交由 OpenCV 读取。
    * **Web Camera**：支持通过 URL 串流读取网络相机数据。
* **ROS 2 动态参数**：支持在运行时动态修改 FPS、曝光模式、曝光时间以及增益（非 ZMQ 模式下生效）。
* **Camera Info**：支持通过 `camera_info_manager` 加载标准的相机标定 YAML 文件。

## 依赖

在编译和运行此包之前，请确保已安装以下依赖：

* ROS 2 (Foxy / Humble / Iron 等)
* OpenCV (`libopencv-dev`)
* ZeroMQ (`libzmq3-dev` / `cppzmq-dev`)
* ROS 2 核心组件：`rclcpp`, `sensor_msgs`, `image_transport`, `cv_bridge`, `camera_info_manager`

安装额外的系统依赖：
```bash
sudo apt update
sudo apt install libzmq3-dev libcppzmq-dev
```

## 编译

将此包放置在 ROS 2 工作空间的 `src` 目录下，然后使用 `colcon` 进行编译：

```bash
cd ~/ros2_ws
colcon build --packages-select usb_camera
source install/setup.bash
```

## 节点参数配置 (`camera_params.yaml`)

核心配置文件为 `camera_params.yaml`。您可以根据需要调整数据源。

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `use_zmq` | bool | `False` | 是否启用 ZMQ 接收流。如果为 `True`，将忽略本地 USB 和 Web 相机配置。 |
| `zmq_endpoint` | string | `tcp://127.0.0.1:5134` | ZMQ 订阅者的地址。 |
| `web_cam` | bool | `False` | 是否使用网络摄像头。如果为 `True`，将忽略 `camera_index`。 |
| `web_cam_url` | string | `http://.../video` | 网络相机的 URL 地址。 |
| `camera_index` | int | `0` | USB 摄像头设备号（对应 `/dev/videoX`）。 |
| `pixel_format` | string | `MJPG` | V4L2 底层格式，支持 `MJPG`, `YUYV`, `NV12`, `RGB3`。 |
| `encoding_str` | string | `bgr8` | 发布至 ROS 的图像编码（Mac 推荐 `rgb8`，其他推荐 `bgr8`）。 |
| `frame_width` | int | `1920` | 图像宽度。**注意：ZMQ 模式下必须与发送端完全一致**。 |
| `frame_height` | int | `1080` | 图像高度。**注意：ZMQ 模式下必须与发送端完全一致**。 |
| `fps` | double | `120.0` | 目标帧率，用于控制 ROS 定时器。 |
| `exposure_auto` | bool | `True` | 自动曝光开关。 |
| `exposure_time` | int | `800` | 手动曝光时间（`exposure_auto` 为 `False` 时生效）。 |
| `gain` | int | `128` | 相机增益。 |
| `camera_info_url` | string | `""` | 相机内参文件路径，例如 `file:///home/user/calib.yaml`。 |

> **⚠️ 关键警告 (针对 ZMQ 模式)**：
> 当 `use_zmq: True` 时，C++ 节点会对接收到的字节流进行严格的内存大小校验 (`width * height * 3`)。`frame_width` 和 `frame_height` **必须**与外部相机（如 Python 发送端）的实际分辨率严格对齐，否则节点将丢弃异常尺寸的帧以防止内存越界。

## 运行节点

运行节点并加载参数文件：

```bash
ros2 run usb_camera usb_camera_node --ros-args --params-file /path/to/your/camera_params.yaml
```

*(如果配置了 launch 文件，也可以直接通过 launch 启动。)*

## 发布的 ROS 话题

* `/image_raw` (`sensor_msgs/msg/Image`): 原始图像数据。
* `/camera_info` (`sensor_msgs/msg/CameraInfo`): 相机内参及畸变系数（需配置 `camera_info_url`）。

## 动态参数调整 (RQT)

当不使用 ZMQ 而是直接挂载物理相机时，您可以通过 `rqt` 工具动态调整相机参数：

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

支持动态修改的参数包括：
* `fps`
* `exposure_auto`
* `exposure_time`
* `gain`