#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdlib> // Necesario para setenv()

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_driver") {
        // 1. Configuración de entorno DENTRO del nodo
        // Esto evita tener que exportar las variables manualmente cada vez
        setenv("LD_LIBRARY_PATH", "/usr/local/lib/aarch64-linux-gnu", 1);
        setenv("GST_PLUGIN_PATH", "/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0", 1);
        setenv("LIBCAMERA_IPA_MODULE_PATH", "/usr/local/lib/aarch64-linux-gnu/libcamera/ipa", 1);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // 2. Definir la pipeline que YA probamos y sabemos que funciona
        // Usamos NV12 por eficiencia y videoconvert para pasar a BGR
        std::string pipeline = 
            "libcamerasrc ! "
            "video/x-raw, width=640, height=480, format=NV12 ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=1";

        // 3. Abrir con el backend de GStreamer
        cap_.open(pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Error: No se pudo abrir la cámara con GStreamer.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Cámara FutBot iniciada en /camera/image_raw");
            timer_ = this->create_wall_timer(33ms, std::bind(&CameraNode::timer_callback, this));
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

        // ROS usa bgr8 para OpenCV por defecto
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