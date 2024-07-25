#include "gmsl_ros2_driver/gmsl_ros2_driver.hpp"

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

//
#include <opencv2/opencv.hpp>

//
#include <atomic>
#include <thread>

#include "MvGmslCamera.h"

using namespace std::chrono_literals;

namespace gmsl_ros2_driver {

struct GmslDriver::Impl {
    std::vector<image_transport::Publisher> img_pubs;

    struct sync_out_a_cfg_client_t stCameraCfgSend = {};
    camera_context_t ctx[8] = {};
    std::unique_ptr<miivii::MvGmslCamera> p_mvcam;

    std::atomic_bool rx_req_stop = false;
    std::thread rx_thread;

    std::string out_fmt;
    int out_w, out_h;
    int camera_num;
};

GmslDriver::GmslDriver(const rclcpp::NodeOptions &options)
    : rclcpp::Node("gmsl_driver_node", options), pImpl(std::make_unique<Impl>()) {
#define PARAM(var, ...)                                               \
    [this]() {                                                        \
        declare_parameter<decltype(pImpl->var)>(#var, ##__VA_ARGS__); \
        get_parameter(#var, pImpl->var);                              \
        RCLCPP_INFO_STREAM(get_logger(), #var ": " << pImpl->var);    \
    }()
    PARAM(out_fmt, "UYVY");
    PARAM(out_w, 1280);
    PARAM(out_h, 720);
    PARAM(camera_num, 5);
#undef PARAM

    if (pImpl->camera_num <= 0 || pImpl->camera_num > 8) {
        RCLCPP_ERROR(get_logger(), "camera_num = %d, which should be 1~8", pImpl->camera_num);
        return;
    }

    rclcpp::QoS qos(1);
    pImpl->img_pubs.resize(pImpl->camera_num);
    for (int i = 0; i < pImpl->camera_num; i++) {
        pImpl->img_pubs[i] = image_transport::create_publisher(this, "video" + std::to_string(i) + "/raw/image",
                                                               qos.get_rmw_qos_profile());
    }

    pImpl->stCameraCfgSend.async_camera_num = 0;
    pImpl->stCameraCfgSend.async_freq = 0;
    pImpl->stCameraCfgSend.async_camera_bit_draw = 0;
    pImpl->stCameraCfgSend.sync_camera_num = 8;
    pImpl->stCameraCfgSend.sync_freq = 30;
    pImpl->stCameraCfgSend.sync_camera_bit_draw = 0xff;

    std::string dev_node = "/dev/video0";
    char dev_node_tmp = '0';
    for (int i = 0; i < pImpl->camera_num; i++) {
        dev_node[10] = dev_node_tmp + i;
        pImpl->ctx[i].dev_node = dev_node;
        pImpl->ctx[i].camera_fmt_str = "UYVY";
        pImpl->ctx[i].output_fmt_str = pImpl->out_fmt;
        pImpl->ctx[i].cam_w = 1920;
        pImpl->ctx[i].cam_h = 1080;
        pImpl->ctx[i].out_w = pImpl->out_w;
        pImpl->ctx[i].out_h = pImpl->out_h;
    }
    pImpl->p_mvcam = std::make_unique<miivii::MvGmslCamera>(pImpl->ctx, pImpl->camera_num, pImpl->stCameraCfgSend);
    pImpl->rx_thread = std::thread([this]() {
        std::vector<uint8_t> pData{pImpl->camera_num};
        std::uint64_t timestamp = 0;
        std::vector<std::thread> threads{pImpl->camera_num};
        while (true) {
            pImpl->p_mvcam->GetImagePtr(pData.data(), timestamp, pImpl->camera_num, "NONE");
            for (int i = 0; i < pImpl->camera_num; i++) {
                if (threads[i].joinable()) threads[i].join();
            }
            for (int i = 0; i < pImpl->camera_num; i++) {
                threads[i] = std::thread([this, timestamp, i, data = pData[i]]() {
                    auto p_msg = std::make_unique<sensor_msgs::msg::Image>();
                    p_msg->header.frame_id = "video" + std::to_string(i);
                    p_msg->header.stamp.sec = timestamp / 1000000000ull;
                    p_msg->header.stamp.nanosec = timestamp % 1000000000ull;
                    p_msg->width = pImpl->out_w;
                    p_msg->height = pImpl->out_h;
                    if (pImpl->ctx[i].output_fmt_str == "UYVY") {
                        p_msg->step = p_msg->width * 2;
                        p_msg->encoding = sensor_msgs::image_encodings::YUV422;
                    } else if (pImpl->ctx[i].output_fmt_str == "ABGR32") {
                        RCLCPP_WARN(get_logger(), "unsupport format: %s", pImpl->ctx[i].output_fmt_str.c_str());
                        return;
                    } else if (pImpl->ctx[i].output_fmt_str == "BGRA32") {
                        p_msg->step = p_msg->width * 4;
                        p_msg->encoding = sensor_msgs::image_encodings::BGRA8;
                    } else {
                        RCLCPP_WARN(get_logger(), "unsupport format: %s", pImpl->ctx[i].output_fmt_str.c_str());
                        return;
                    }
                    p_msg->data.resize(p_msg->height * p_msg->step);
                    std::copy_n(data, p_msg->data.size(), p_msg->data.data());
                    pImpl->img_pubs[i].publish(std::move(p_msg));
                });
            }
        }
    });
}

GmslDriver::~GmslDriver() {
    if (pImpl) {
        pImpl->rx_req_stop = true;
        if (pImpl->rx_thread.joinable()) pImpl->rx_thread.join();
    }
}

}  // namespace gmsl_ros2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gmsl_ros2_driver::GmslDriver)