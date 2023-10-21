#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node 
{
public:
  SensorPublisher () :
    Node("sensor_emulator")
  {
    pub_sonar_ = this->create_publisher<sensor_msgs::msg::Range>("sonar", 10);
    pub_ir_ = this->create_publisher<sensor_msgs::msg::Range>("ir", 10);

    timer_sonar_ = this->create_wall_timer(20ms, [&](){timer_sonar();});
  //  timer_ir_ = this->create_wall_timer(20ms, [&](){timer_ir();});
  }

  void timer_sonar () {
    sensor_msgs::msg::Range message;

    message.radiation_type = 0;
    message.field_of_view = 0.1f;
    message.min_range = 0.05;
    message.max_range = 3.0f;
    message.range = 3.0f;
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "sonar_"+std::to_string(index)+"_link";
    pub_sonar_->publish(message);

    index++;
    if (index == 17) {
      index = 1;
    }
  }

  void timer_ir () {
    sensor_msgs::msg::Range message;

    message.radiation_type = 1;
    message.field_of_view = 0.1f;
    message.min_range = 0.05;
    message.max_range = 3.0f;
    message.range = index%2 ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "ir_"+std::to_string(index)+"_link";
    pub_ir_->publish(message);
    
    index++;
    if (index == 17) {
      index = 1;
    }
  }
private:
  int index = 0;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_sonar_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_ir_;
  rclcpp::TimerBase::SharedPtr timer_sonar_;
  rclcpp::TimerBase::SharedPtr timer_ir_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPublisher>());
  rclcpp::shutdown();
  return 0;
}