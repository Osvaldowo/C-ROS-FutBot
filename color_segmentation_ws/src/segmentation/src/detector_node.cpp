#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode() : Node("detector_vision") {
        // QoS para la cámara (SensorData)
        rclcpp::SensorDataQoS qos_profile_sensor;

        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 
            qos_profile_sensor,
            std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

        // QoS para Calibración
        rclcpp::QoS qos_profile_transient(1);
        qos_profile_transient.transient_local();
        
        sub_calibration_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/vision/calibration_data", qos_profile_transient,
            std::bind(&DetectorNode::calibration_callback, this, std::placeholders::_1));

        // Publicadores
        pub_point_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/ball_position", 10);
        pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_output", 10);

        // Valores iniciales por defecto (verde/amarillo de prueba)
        lower_hsv_ = cv::Scalar(0, 100, 100);
        upper_hsv_ = cv::Scalar(179, 255, 255);
        z_filtrada_ = 0.0f;

        RCLCPP_INFO(this->get_logger(), "Detector C-ROS reactivado con CLAHE y Filtro Alpha.");
    }

private:
    void calibration_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            lower_hsv_ = cv::Scalar(msg->data[0], msg->data[1], msg->data[2]);
            upper_hsv_ = cv::Scalar(msg->data[3], msg->data[4], msg->data[5]);
            RCLCPP_INFO(this->get_logger(), "HSV actualizado.");
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 1. Convertir imagen ROS a OpenCV
            cv::Mat raw_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // 2. Rectificación de lente
            cv::Mat frame_rect;
            cv::undistort(raw_frame, frame_rect, camera_matrix, dist_coeffs);

            // 3. Normalización de Luz (CLAHE)
            cv::Mat lab_image;
            cv::cvtColor(frame_rect, lab_image, cv::COLOR_BGR2Lab);
            std::vector<cv::Mat> lab_planes(3);
            cv::split(lab_image, lab_planes);
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
            clahe->apply(lab_planes[0], lab_planes[0]);
            cv::Mat frame_clahe;
            cv::merge(lab_planes, lab_image);
            cv::cvtColor(lab_image, frame_clahe, cv::COLOR_Lab2BGR);

            // 4. Espacio de Color HSV y Limpieza
            cv::Mat hsv, mask;
            cv::cvtColor(frame_clahe, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, lower_hsv_, upper_hsv_, mask);
            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

            // 5. Contornos
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

            // 6. Lógica de Profundidad (Z)
            if (max_idx >= 0) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contours[max_idx], center, radius);
                
                float diameter_pixels = radius * 2.0f;
                // Usando constante focal empírica y pelota de golf
                // float z_raw = (4.27f * 634.66f) / diameter_pixels;
                float z_raw = (4.27f * 470.49f) / diameter_pixels;

                // Filtro Alpha
                if (z_filtrada_ == 0.0f) z_filtrada_ = z_raw;
                else z_filtrada_ = (ALPHA * z_raw) + (1.0f - ALPHA) * z_filtrada_;

                // Publicar Punto 3D
                auto point_msg = geometry_msgs::msg::Point();
                point_msg.x = center.x;
                point_msg.y = center.y;
                point_msg.z = z_filtrada_;
                pub_point_->publish(point_msg);

                // Dibujar UI en video
                cv::circle(frame_clahe, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                std::string info = "Z: " + std::to_string(static_cast<int>(z_filtrada_)) + " cm";
                cv::putText(frame_clahe, info, cv::Point(center.x + 10, center.y), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            } else {
                z_filtrada_ = 0.0f; 
            }

            // 7. Publicar imagen Debug
            sensor_msgs::msg::Image::SharedPtr debug_msg = 
                cv_bridge::CvImage(msg->header, "bgr8", frame_clahe).toImageMsg();
            pub_debug_->publish(*debug_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_calibration_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_point_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
    
    cv::Scalar lower_hsv_, upper_hsv_;
    float z_filtrada_;
    const float ALPHA = 0.2f;

    // Constantes de Calibración rescatadas
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 470.49451, 0, 339.39478, 0, 499.34195, 185.99863, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << -0.421, 0.185, 0.0, 0.0, 0.0); 
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}