// camera_cpp/src/array_subscriber.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class ArraySubscriber : public rclcpp::Node {
public:
    ArraySubscriber() : Node("array_subscriber_cpp") {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "array_topic", 10, std::bind(&ArraySubscriber::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Array Subscriber Node started (C++)");
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received: ");
        for (auto val : msg->data) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArraySubscriber>());
    rclcpp::shutdown();
    return 0;
}
