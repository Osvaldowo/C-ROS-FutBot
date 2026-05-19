#include <chrono>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class BallPlannerNode : public rclcpp::Node
{
public:
  BallPlannerNode() : Node("ball_planner_node"), has_odom_(false)
  {

    // Parámetros de la cámara (HFOV de tu OV5647 = 1.1944 radianes)
    this->declare_parameter("camera_hfov", 1.1944);
    this->declare_parameter("image_width", 640.0);

    // Publicador de la trayectoria (Path)
    pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    // Suscriptores
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&BallPlannerNode::odom_callback, this, std::placeholders::_1));

    sub_vision_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/vision/ball_position", 10, std::bind(&BallPlannerNode::vision_callback, this, std::placeholders::_1));

    sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&BallPlannerNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Planificador de Trayectorias iniciado. Esperando visión y odometría...");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    robot_theta_ = std::atan2(siny_cosp, cosy_cosp);

    has_odom_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!has_odom_) return;

    double force_x = 0.0;
    double force_y = 0.0;
    double k_rep = 0.05; // Ganancia de repulsión (ajustar según el peso deseado)
    double safe_dist = 0.4; // Ignorar objetos a más de 40 cm

    // Recorrer todos los rayos del LiDAR
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      
      // Si el láser lee un dato válido y está peligrosamente cerca
      if (r > msg->range_min && r < safe_dist) {
        // Calcular el ángulo global de este rayo
        double angle_local = msg->angle_min + i * msg->angle_increment;
        double angle_global = robot_theta_ + angle_local;

        // Calcular la magnitud de repulsión (inversamente proporcional al cuadrado de la distancia)
        double magnitude = k_rep / (r * r);

        // Sumar el vector de fuerza (hacia el lado opuesto del obstáculo, por eso el '-')
        force_x -= magnitude * std::cos(angle_global);
        force_y -= magnitude * std::sin(angle_global);
      }
    }

    // Guardar la fuerza resultante para usarla en la curva de Bezier
    rep_x_ = force_x;
    rep_y_ = force_y;
  }

  void vision_callback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    if (!has_odom_) return;

    double hfov = this->get_parameter("camera_hfov").as_double();
    double width = this->get_parameter("image_width").as_double();

    // 1. Calcular coordenadas globales de la pelota
    double center_x = width / 2.0;
    double pixel_error = center_x - msg->x;
    double angle_to_ball_local = (pixel_error / width) * hfov;
    double distance_m = msg->z / 100.0;

    double global_ball_x = robot_x_ + distance_m * std::cos(robot_theta_ + angle_to_ball_local);
    double global_ball_y = robot_y_ + distance_m * std::sin(robot_theta_ + angle_to_ball_local);

    // 2. Construir la trayectoria (Path)
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = this->now();

    // 3. Puntos de Control de Bezier
    double P0_x = robot_x_;
    double P0_y = robot_y_;
    
    // P1: Proyectado frente al robot (50% de la distancia total) para forzar tangencia
    double k = distance_m * 0.5; 
    double P1_x = robot_x_ + k * std::cos(robot_theta_) + rep_x_; // Agregar la fuerza de repulsión al punto de control
    double P1_y = robot_y_ + k * std::sin(robot_theta_) + rep_y_; // Agregar la fuerza de repulsión al punto de control

    // P2 y P3: La posición de la pelota
    double P2_x = global_ball_x;
    double P2_y = global_ball_y;
    double P3_x = global_ball_x;
    double P3_y = global_ball_y;

    // 4. Generar los puntos iterando sobre t (de 0.0 a 1.0)
    int num_waypoints = 20; // 20 puntos para una curva muy suave
    for (int i = 0; i <= num_waypoints; ++i) {
      double t = (double)i / num_waypoints;
      double u = 1.0 - t;
      
      // Ecuación de Bezier Cúbica
      double x = (u*u*u)*P0_x + 3*(u*u)*t*P1_x + 3*u*(t*t)*P2_x + (t*t*t)*P3_x;
      double y = (u*u*u)*P0_y + 3*(u*u)*t*P1_y + 3*u*(t*t)*P2_y + (t*t*t)*P3_y;

      geometry_msgs::msg::PoseStamped pose_inter;
      pose_inter.pose.position.x = x;
      pose_inter.pose.position.y = y;
      path_msg.poses.push_back(pose_inter);
    }

    pub_path_->publish(path_msg);
  }

  bool has_odom_;
  double robot_x_, robot_y_, robot_theta_;
  double rep_x_ = 0.0;
  double rep_y_ = 0.0;
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_vision_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallPlannerNode>());
  rclcpp::shutdown();
  return 0;
}