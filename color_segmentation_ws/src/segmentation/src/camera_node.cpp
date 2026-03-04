#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_driver") {
        // Publicamos en 'camera/image_raw' (estándar en ROS)
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
        timer_ = this->create_wall_timer(33ms, std::bind(&CameraNode::timer_callback, this)); // ~30 FPS

        // Intentar abrir cámara (índice 0)
        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error: No se detecta la cámara.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Cámara iniciada. Publicando en /camera/image_raw");
        }
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_optical_frame";

        sensor_msgs::msg::Image::SharedPtr msg = 
            cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        
        publisher_->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
