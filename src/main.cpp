#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <aim_assistant/aim_assistant_component.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<abu2023::AimAssistant>());
    rclcpp::shutdown();
    return 0;
}