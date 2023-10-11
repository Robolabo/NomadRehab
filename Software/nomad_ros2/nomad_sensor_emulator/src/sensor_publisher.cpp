#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node 
{
public:
  SensorPublisher () :
    Node("sensor_emulator")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Range>("sonar", 10);
    timer_ = this->create_wall_timer(20ms, [&](){timer_callback();});
  }

  void timer_callback () {
    sensor_msgs::msg::Range message;

    message.radiation_type = 0;
    message.field_of_view = 0.1f;
    message.min_range = 0.05;
    message.max_range = 3.0f;
    message.range = 3.0f;
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "sonar_"+std::to_string(index)+"_link";
    pub_->publish(message);
    message.header.frame_id = "ir_"+std::to_string(index)+"_link";
    pub_->publish(message);

    index++;
    if (index == 17) {
      index = 1;
    }
  }
private:
  int index = 0;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPublisher>());
  rclcpp::shutdown();
  return 0;
}