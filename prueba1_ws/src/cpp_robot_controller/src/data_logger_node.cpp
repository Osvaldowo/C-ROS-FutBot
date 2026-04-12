#include <fstream>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class DataLogger : public rclcpp::Node
{
public:
  DataLogger() : Node("data_logger")
  {
    // Nombre del archivo con timestamp para no sobrescribir
    std::string filename = "robot_data_log.csv";
    csv_file_.open(filename);

    // Encabezados del CSV (compatibles con Excel)
    csv_file_ << "timestamp,v_motor_left,v_motor_right,odom_x,odom_y,odom_theta,linear_v,angular_v\n";

    // Suscripciones
    sub_vel_left_ = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_vel_left", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { v_l_ = msg->data; });
    
    sub_vel_right_ = this->create_subscription<std_msgs::msg::Float32>(
      "/motor_vel_right", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) { v_r_ = msg->data; });

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&DataLogger::odom_callback, this, std::placeholders::_1));

    // Timer para guardar datos a 10Hz (ajustable)
    timer_ = this->create_wall_timer(100ms, std::bind(&DataLogger::log_data, this));

    RCLCPP_INFO(this->get_logger(), "Logger iniciado. Guardando en: %s", filename.c_str());
  }

  // Cerrar el archivo correctamente al destruir el nodo
  ~DataLogger() {
    if (csv_file_.is_open()) csv_file_.close();
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    curr_odom_ = *msg;
    
    // Extraer theta del cuaternión
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    curr_theta_ = std::atan2(siny_cosp, cosy_cosp);
  }

  void log_data()
  {
    if (!csv_file_.is_open()) return;

    // Obtener tiempo actual en segundos
    double ts = this->now().seconds();

    // Escribir línea en el CSV
    csv_file_ << ts << ","
              << v_l_ << ","
              << v_r_ << ","
              << curr_odom_.pose.pose.position.x << ","
              << curr_odom_.pose.pose.position.y << ","
              << curr_theta_ << ","
              << curr_odom_.twist.twist.linear.x << ","
              << curr_odom_.twist.twist.angular.z << "\n";
  }

  std::ofstream csv_file_;
  float v_l_ = 0.0, v_r_ = 0.0;
  double curr_theta_ = 0.0;
  nav_msgs::msg::Odometry curr_odom_;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vel_left_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vel_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DataLogger>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}