#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>

#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

class USBCameraNode : public rclcpp::Node
{
public:
    USBCameraNode() : Node("usb_camera_node")
    {
        // ===== 参数声明 =====
        this->declare_parameter<bool>("use_zmq", false);
        this->declare_parameter<std::string>("zmq_endpoint", "tcp://127.0.0.1:5134");

        this->declare_parameter<bool>("web_cam", false);
        this->declare_parameter<std::string>("web_cam_url", "http://127.0.0.1:4090/video");

        this->declare_parameter<int>("camera_index", 0);
        this->declare_parameter<bool>("exposure_auto", true);
        this->declare_parameter<int>("exposure_time", 800);
        this->declare_parameter<double>("fps", 120.0);
        this->declare_parameter<int>("gain", 128);
        this->declare_parameter<int>("frame_width", 1920);
        this->declare_parameter<int>("frame_height", 1080);
        this->declare_parameter<std::string>("camera_name", "usb_camera");
        this->declare_parameter<std::string>("camera_info_url", "");
        this->declare_parameter<std::string>("pixel_format", "MJPG");
        this->declare_parameter<std::string>("encoding_str", "bgr8");

        // 读取参数
        this->get_parameter_or("use_zmq", use_zmq_, false);
        this->get_parameter_or("zmq_endpoint", zmq_endpoint_, std::string("tcp://127.0.0.1:5134"));

        this->get_parameter_or("frame_width", frame_width_, 1920);
        this->get_parameter_or("frame_height", frame_height_, 1080);
        this->get_parameter_or("web_cam", web_cam_, false);
        this->get_parameter_or("web_cam_url", web_cam_url_, std::string("http://127.0.0.1:4090/video"));
        this->get_parameter_or("camera_index", camera_index_, 0);
        this->get_parameter_or("fps", fps_, 120.0);
        this->get_parameter_or("exposure_auto", exposure_auto_, true);
        this->get_parameter_or("exposure_time", exposure_time_, 800);
        this->get_parameter_or("gain", gain_, 128);
        this->get_parameter_or("camera_name", camera_name_, std::string("usb_camera"));
        this->get_parameter_or("camera_info_url", camera_info_url_, std::string(""));
        this->get_parameter_or("pixel_format", pixel_format_, std::string("MJPG"));
        this->get_parameter_or("encoding_str", encoding_str_, std::string("bgr8"));

        // ===== CameraInfoManager =====
        cinfo_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_, camera_info_url_);
        if (!camera_info_url_.empty()) {
            if (cinfo_mgr_->loadCameraInfo(camera_info_url_)) {
                RCLCPP_INFO(this->get_logger(), "加载相机标定文件: %s", camera_info_url_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "加载相机标定文件失败: %s", camera_info_url_.c_str());
            }
        }

        // ===== 初始化数据源 =====
        if (use_zmq_) {
            RCLCPP_INFO(this->get_logger(), "视频流源: ZMQ (%s) - 期望接收 %dx%d 原始字节流", 
                        zmq_endpoint_.c_str(), frame_width_, frame_height_);
            zmq_ctx_ = std::make_shared<zmq::context_t>(1);
            zmq_socket_ = std::make_shared<zmq::socket_t>(*zmq_ctx_, zmq::socket_type::sub);
            
            // 开启 Conflate 保证仅保留最新帧
            int conflate = 1;
            zmq_socket_->set(zmq::sockopt::conflate, conflate);
            zmq_socket_->connect(zmq_endpoint_);
            zmq_socket_->set(zmq::sockopt::subscribe, "");
            
            RCLCPP_INFO(this->get_logger(), "ZMQ 订阅者初始化完成，Conflate已开启");
        } 
        else {
            configure_v4l2();
            if (web_cam_) {
                RCLCPP_INFO(this->get_logger(), "使用 Web Camera: %s", web_cam_url_.c_str());
                cap_.open(web_cam_url_);
            } else {
                RCLCPP_INFO(this->get_logger(), "使用 USB Camera");
                cap_.open(camera_index_, cv::CAP_V4L2);
            }

            if (!cap_.isOpened()) {
                RCLCPP_FATAL(this->get_logger(), "无法打开摄像头 %d", camera_index_);
                throw std::runtime_error("Camera open failed");
            }

            cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(pixel_format_[0], pixel_format_[1], pixel_format_[2], pixel_format_[3]));
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
            cap_.set(cv::CAP_PROP_FPS, fps_);

            log_camera_info("OpenCV 初始化后");
            fps_ = cap_.get(cv::CAP_PROP_FPS);
            apply_exposure();
            apply_gain();
        }

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        cinfo_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

        create_timer();

        callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&USBCameraNode::parameters_callback, this, std::placeholders::_1));
    }

private:
    void configure_v4l2()
    {
        if (web_cam_) {
            RCLCPP_INFO(this->get_logger(), "使用 Web Camera: %s", web_cam_url_.c_str());
            cap_.open(web_cam_url_);
        }
        else {
            std::string device = "/dev/video" + std::to_string(camera_index_);
            int fd = open(device.c_str(), O_RDWR);
            if (fd < 0) {
                RCLCPP_ERROR(this->get_logger(), "无法打开设备文件 %s", device.c_str());
                return;
            }

            struct v4l2_format fmt;
            std::memset(&fmt, 0, sizeof(fmt));
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmt.fmt.pix.width = frame_width_;
            fmt.fmt.pix.height = frame_height_;
            fmt.fmt.pix.field = V4L2_FIELD_ANY;

            if (pixel_format_ == "YUYV") {
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            } else if (pixel_format_ == "NV12"){
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
            } else if (pixel_format_ == "RGB3") {
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
            } else {
                fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
            }

            ioctl(fd, VIDIOC_S_FMT, &fmt);

            struct v4l2_streamparm parm;
            std::memset(&parm, 0, sizeof(parm));
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            parm.parm.capture.timeperframe.numerator = 1;
            parm.parm.capture.timeperframe.denominator = static_cast<unsigned int>(fps_);
            ioctl(fd, VIDIOC_S_PARM, &parm);

            close(fd);
            cap_.open(camera_index_, cv::CAP_V4L2);
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "cv::VideoCapture 初始化失败！");
        }
    }

    void set_v4l2_control(int control_id, int value)
    {
        if (!web_cam_){
            std::string device = "/dev/video" + std::to_string(camera_index_);
            int fd = open(device.c_str(), O_RDWR);
            if (fd < 0) return;
            struct v4l2_control ctrl;
            ctrl.id = control_id;
            ctrl.value = value;
            ioctl(fd, VIDIOC_S_CTRL, &ctrl);
            close(fd);
        }
    }

    void apply_exposure()
    {
        if (exposure_auto_) {
            if (!cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 3)) {
                set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO);
            }
        } else {
            if (!cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1)) {
                set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
            }
            if (!cap_.set(cv::CAP_PROP_EXPOSURE, exposure_time_)) {
                set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_time_);
            }
        }
    }

    void apply_gain()
    {
        if (!cap_.set(cv::CAP_PROP_GAIN, gain_)) {
            set_v4l2_control(V4L2_CID_GAIN, gain_);
        }
    }

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params) {
            if (param.get_name() == "fps") {
                double req_fps = param.as_double();
                if (!use_zmq_) {
                    cap_.set(cv::CAP_PROP_FPS, req_fps);
                    fps_ = cap_.get(cv::CAP_PROP_FPS);
                } else {
                    fps_ = req_fps;
                }
                create_timer();
            }
            else if (param.get_name() == "exposure_auto" && !use_zmq_) {
                exposure_auto_ = param.as_bool();
                apply_exposure();
            }
            else if (param.get_name() == "exposure_time" && !use_zmq_) {
                exposure_time_ = param.as_int();
                if (!exposure_auto_) apply_exposure();
            }
            else if (param.get_name() == "gain" && !use_zmq_) {
                gain_ = param.as_int();
                apply_gain();
            }
        }
        return result;
    }

    void log_camera_info(const std::string &prefix)
    {
        double rw = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double rh = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double rfps = cap_.get(cv::CAP_PROP_FPS);
        int fourcc = static_cast<int>(cap_.get(cv::CAP_PROP_FOURCC));
        char fourcc_str[] = {
            static_cast<char>(fourcc & 0XFF),
            static_cast<char>((fourcc >> 8) & 0XFF),
            static_cast<char>((fourcc >> 16) & 0XFF),
            static_cast<char>((fourcc >> 24) & 0XFF),
            0
        };
        RCLCPP_INFO(this->get_logger(),
            "%s: %dx%d @ %.2f fps, 格式=%s",
            prefix.c_str(), (int)rw, (int)rh, rfps, fourcc_str);
    }

    // ========== 采帧 ==========
    void capture_frame()
    {
        cv::Mat frame;

        if (use_zmq_) {
            zmq::message_t msg;
            auto res = zmq_socket_->recv(msg, zmq::recv_flags::dontwait);
            if (!res || !res.has_value()) {
                return; // 当前周期无新消息
            }

            // 严格校验字节大小以防内存溢出 (宽度 * 高度 * 3通道)
            size_t expected_size = static_cast<size_t>(frame_width_ * frame_height_ * 3);
            if (msg.size() != expected_size) {
                RCLCPP_WARN(this->get_logger(), "收到尺寸异常的字节流！预期大小: %zu，实际大小: %zu。请检查 yaml 中的尺寸设置是否与发送端完全一致。", expected_size, msg.size());
                return;
            }

            // 直接将字节流映射为 OpenCV 图像矩阵，并克隆到新内存中以防 msg 被释放
            frame = cv::Mat(frame_height_, frame_width_, CV_8UC3, msg.data()).clone();

        } else {
            if (!cap_.read(frame)) {
                RCLCPP_WARN(this->get_logger(), "捕获帧失败 (Frame Dropped or Timeout)");
                return;
            }
        }

        auto stamp = this->now();
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = "camera_optical_frame";
        
        if (encoding_str_ == "rgb8"){
            cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        }
        auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*img_msg);

        auto ci = cinfo_mgr_->getCameraInfo();
        ci.header = header;
        cinfo_pub_->publish(ci);
    }

    void create_timer()
    {
        if (fps_ <= 0.0) fps_ = 30.0;
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps_));
        timer_ = this->create_wall_timer(period, std::bind(&USBCameraNode::capture_frame, this));
    }

    // 成员
    bool use_zmq_;
    std::string zmq_endpoint_;
    std::shared_ptr<zmq::context_t> zmq_ctx_;
    std::shared_ptr<zmq::socket_t> zmq_socket_;

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cinfo_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_mgr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    int camera_index_;
    int frame_width_;
    int frame_height_;
    double fps_;
    bool exposure_auto_;
    int exposure_time_;
    int gain_;
    std::string camera_name_;
    std::string camera_info_url_;
    std::string pixel_format_;
    std::string encoding_str_;

    bool web_cam_;
    std::string web_cam_url_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<USBCameraNode>());
    rclcpp::shutdown();
    return 0;
}