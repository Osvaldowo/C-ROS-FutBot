#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class FigureEightSequencer : public rclcpp::Node
{
public:
  FigureEightSequencer()
  : Node("figure_eight_sequencer"), state_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Guardamos el tiempo de inicio
    start_time_ = this->now();

    // Timer a 10Hz (cada 100ms)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&FigureEightSequencer::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Iniciando Trayectoria en Forma de 8...");
    RCLCPP_INFO(this->get_logger(), "Trazando Círculo Izquierdo");
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    rclcpp::Time current_time = this->now();
    
    // Calculamos cuántos segundos han pasado desde que inició el estado actual
    double elapsed_seconds = (current_time - start_time_).seconds();

    // El tiempo para dar una vuelta completa (2 * PI radianes) a 1 rad/s
    double time_per_circle = 2.0 * M_PI; // ~6.283 segundos

    // Cambiamos de estado cada vez que se completa un círculo
    if (elapsed_seconds >= time_per_circle) {
      state_ = (state_ == 0) ? 1 : 0; // Alternar entre 0 y 1
      start_time_ = current_time;     // Reiniciar el cronómetro
      
      if (state_ == 0) {
        RCLCPP_INFO(this->get_logger(), "Trazando Círculo Izquierdo");
      } else {
        RCLCPP_INFO(this->get_logger(), "Trazando Círculo Derecho");
      }
    }

    // --- COMANDOS DE MOVIMIENTO ---
    msg.linear.x = 400.0; // Avanzar siempre a 400 mm/s (Tu ESP32 lo espera así)
    
    if (state_ == 0) {
      msg.angular.z = 1.0;  // Girar a la izquierda (1 rad/s)
    } else {
      msg.angular.z = -1.0; // Girar a la derecha (-1 rad/s)
    }

    publisher_->publish(msg);
  }

  int state_; // 0 = Círculo Izquierdo, 1 = Círculo Derecho
  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FigureEightSequencer>());
  rclcpp::shutdown();
  return 0;
}