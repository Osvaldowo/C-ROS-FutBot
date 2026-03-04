#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp> // <--- 1. Indispensable para QoS
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

// Variables globales para los sliders
int low_H = 0, low_S = 100, low_V = 100;
int high_H = 179, high_S = 255, high_V = 255;

class CalibratorNode : public rclcpp::Node {
public:
    CalibratorNode() : Node("calibrador_hsv") {
        
        // --- 2. EL ARREGLO MÁGICO: SensorDataQoS ---
        // Esto le dice al nodo: "Acepta datos aunque se pierdan paquetes" (Best Effort)
        // Es necesario porque los drivers de cámara suelen publicar así.
        rclcpp::SensorDataQoS qos_profile_sensor;

        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 
            qos_profile_sensor, // <--- Aquí aplicamos el perfil compatible
            std::bind(&CalibratorNode::image_callback, this, std::placeholders::_1));

        // Configuración del Publicador (Transient Local para guardar datos al cerrar)
        rclcpp::QoS qos_profile_transient(1); 
        qos_profile_transient.transient_local();

        pub_calibration_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/calibration_data", qos_profile_transient);

        // Ventana y Sliders
        cv::namedWindow("Calibracion", cv::WINDOW_NORMAL);
        cv::resizeWindow("Calibracion", 400, 300);
        cv::createTrackbar("Low H", "Calibracion", &low_H, 179);
        cv::createTrackbar("High H", "Calibracion", &high_H, 179);
        cv::createTrackbar("Low S", "Calibracion", &low_S, 255);
        cv::createTrackbar("High S", "Calibracion", &high_S, 255);
        cv::createTrackbar("Low V", "Calibracion", &low_V, 255);
        cv::createTrackbar("High V", "Calibracion", &high_V, 255);

        RCLCPP_INFO(this->get_logger(), "Calibrador iniciado. Esperando video...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv, mask, resized_mask;

            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), mask);

            // --- NUEVO: Reducir la imagen al 50% ---
            // Esto hará que la ventana se ajuste a un tamaño mucho más cómodo
            cv::resize(mask, resized_mask, cv::Size(), 0.5, 0.5); 

            // Mostramos la imagen ya pequeña
            cv::imshow("Calibracion", resized_mask);
            
            // Publicar datos
            std_msgs::msg::Int32MultiArray calib_msg;
            calib_msg.data = {low_H, low_S, low_V, high_H, high_S, high_V};
            pub_calibration_->publish(calib_msg);

            // Refrescar ventana (waitKey es el corazón de la GUI)
            char key = (char)cv::waitKey(1);
            if (key == 27 || cv::getWindowProperty("Calibracion", cv::WND_PROP_VISIBLE) < 1) {
                RCLCPP_INFO(this->get_logger(), "Guardando y saliendo...");
                rclcpp::shutdown();
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_calibration_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibratorNode>());
    rclcpp::shutdown();
    return 0;
}
