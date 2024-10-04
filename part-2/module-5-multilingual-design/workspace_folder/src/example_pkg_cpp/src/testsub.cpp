/*
Author: Daniel Yanke
Date: 10/3/2024
*/

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "example_test_interfaces/msg/example_test.hpp"
using std::placeholders::_1;

class CppSubscriber : public rclcpp::Node{
public:
    CppSubscriber() : Node("cpp_subscriber"){
        subscription_ = this->create_subscription<example_test_interfaces::msg::ExampleTest>(
            "test_topic", 10, std::bind(&CppSubscriber::topic_callback, this, _1)
        );
    }
private:
    void topic_callback(const example_test_interfaces::msg::ExampleTest & msg) const{
        RCLCPP_INFO(this->get_logger(), "I heard: '%s', '%s' from %s node", msg.unity_id.c_str(), std::to_string(msg.seq_number).c_str(), msg.language.c_str());
    }
    rclcpp::Subscription<example_test_interfaces::msg::ExampleTest>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppSubscriber>());
    rclcpp::shutdown();
    return 0;
}