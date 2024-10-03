#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "example_test_interfaces/msg/ExampleTest.hpp"

class CppPublisher : public rclcpp::Node{
public:
    CppPublisher() : Node("cpp_publisher"), count_(0){
        publisher_ = this->create_publisher<example_test_interfaces::msg::ExampleTest>("cpp_pub", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&CppPublisher::timer_callback, this))
    }
private:
    void timer_callback(){
        auto message = example_test_interfaces::msg::ExampleTest();
        message.unity_id = "djyanke";
        message.seq_number = count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s', '%s'", message.unity_id.c_str(), std::to_string(message.seq_number).c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}