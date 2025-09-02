#include "loader_sim_laserscan_fusion/scan_fusion_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<loader_sim_laserscan_fusion::LaserScanFusionNode>();
  
  RCLCPP_INFO(node->get_logger(), "Starting laser scan fusion node...");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}