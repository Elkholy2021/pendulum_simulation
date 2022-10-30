#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("water_current_pub"), count_(0)
    {
          declare_parameter("x", 3.0);
          declare_parameter("y", 2.0);
          declare_parameter("z", 0.0);

      publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("water_current", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Vector3();
      double Vx = get_parameter("x").as_double();
      double Vy = get_parameter("y").as_double();
      double Vz = get_parameter("z").as_double();

      message.x = Vx;
      message.y = Vy;
      message.z = Vz;
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}