#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp> // <--- 1. IMPORTANTE: Incluir librería de QoS
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode() : Node("detector_vision") {
        
        // --- 2. CORRECCIÓN: Configurar QoS para escuchar a la cámara ---
        rclcpp::SensorDataQoS qos_profile_sensor;

        // Suscripción a la cámara con QoS flexible
        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb/image_raw", 
            qos_profile_sensor, // <--- Aquí usamos el perfil compatible
            std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

        // Suscripción a la CALIBRACIÓN (Persistente)
        rclcpp::QoS qos_profile_transient(1);
        qos_profile_transient.transient_local();
        
        sub_calibration_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/vision/calibration_data", qos_profile_transient,
            std::bind(&DetectorNode::calibration_callback, this, std::placeholders::_1));

        // Publicadores
        pub_point_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/ball_position", 10);
        pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_output", 10);

        // Valores por defecto
        lower_hsv_ = cv::Scalar(0, 100, 100);
        upper_hsv_ = cv::Scalar(179, 255, 255);
        
        RCLCPP_INFO(this->get_logger(), "Detector iniciado y escuchando video...");
    }

private:
    void calibration_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            lower_hsv_ = cv::Scalar(msg->data[0], msg->data[1], msg->data[2]);
            upper_hsv_ = cv::Scalar(msg->data[3], msg->data[4], msg->data[5]);
            RCLCPP_INFO(this->get_logger(), "Calibración actualizada.");
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            cv::Mat hsv, mask;

            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, lower_hsv_, upper_hsv_, mask);

            // Limpieza
            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

            // Contornos
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double max_area = 0;
            int max_idx = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > 500 && area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }

            if (max_idx >= 0) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contours[max_idx], center, radius);
                
                auto point_msg = geometry_msgs::msg::Point();
                point_msg.x = center.x;
                point_msg.y = center.y;
                pub_point_->publish(point_msg);

                // DIBUJAR
                cv::circle(frame, center, (int)radius, cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(frame, "Pelota", center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
            }

            // Publicar imagen DEBUG
            pub_debug_->publish(*cv_ptr->toImageMsg());

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_calibration_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_point_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
    
    cv::Scalar lower_hsv_, upper_hsv_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}
