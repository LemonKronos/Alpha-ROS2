#include "rclcpp/rclcpp.hpp"
#include "contact_parser/contact_parser.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto contact_parser_node = std::make_shared<ContactParserNode>();
    rclcpp::spin(contact_parser_node);
    rclcpp::shutdown();
    return 0;
}