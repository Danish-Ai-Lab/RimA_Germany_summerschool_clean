#include "navpi_testing/mock_base.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navpi::MockBase>());
  rclcpp::shutdown();
  return 0;
}
