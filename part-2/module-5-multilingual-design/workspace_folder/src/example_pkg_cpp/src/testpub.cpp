/*
Author: Daniel Yanke
Date: 10/3/2024
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_test_interfaces/msg/example_test.hpp"

class CppPublisher : public rclcpp::Node{
public:
    CppPublisher() : Node("cpp_publisher"), count_(0){
        publisher_ = this->create_publisher<example_test_interfaces::msg::ExampleTest>("test_topic", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CppPublisher::timer_callback, this));
    }
private:
    void timer_callback(){
        auto message = example_test_interfaces::msg::ExampleTest();
        message.unity_id = "djyanke";
        message.seq_number = count_++;
        message.language = "C++";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s', '%s'", message.unity_id.c_str(), std::to_string(message.seq_number).c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<example_test_interfaces::msg::ExampleTest>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppPublisher>());
    rclcpp::shutdown();
    return 0;
}