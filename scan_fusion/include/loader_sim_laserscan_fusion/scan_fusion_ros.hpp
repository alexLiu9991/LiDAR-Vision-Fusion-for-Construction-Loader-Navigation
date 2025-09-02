#ifndef LOADER_SIM_LASERSCAN_FUSION__LASER_SCAN_FUSION_NODE_HPP_
#define LOADER_SIM_LASERSCAN_FUSION__LASER_SCAN_FUSION_NODE_HPP_

#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace loader_sim_laserscan_fusion
{

class LaserScanFusionNode : public rclcpp::Node
{
public:
  explicit LaserScanFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LaserScanFusionNode() = default;

private:
  // Data buffers for approximate synchronization
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> buf1_;
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> buf2_;
  std::mutex mtx_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  std::string scan1_topic_;
  std::string scan2_topic_;
  std::string output_topic_;
  std::string target_frame_;

  double angle_min_;
  double angle_max_;
  double angle_inc_;

  double out_range_min_;
  double out_range_max_;

  std::string fusion_mode_;
  double weight_scan1_;
  double weight_scan2_;

  bool use_inf_;
  bool fill_intensities_;

  int queue_size_;
  double slop_sec_;
  double tf_timeout_sec_;

  // Callback functions
  void scan1_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void scan2_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Helper functions
  void push_and_trim(std::deque<sensor_msgs::msg::LaserScan::SharedPtr>& buf,
                     const sensor_msgs::msg::LaserScan::SharedPtr& msg);
  void try_match_and_process();
  static void erase_msg(std::deque<sensor_msgs::msg::LaserScan::SharedPtr>& buf,
                        const sensor_msgs::msg::LaserScan::SharedPtr& target);
  std::pair<sensor_msgs::msg::LaserScan::SharedPtr, sensor_msgs::msg::LaserScan::SharedPtr>
  find_best_pair();
  bool lookup_transform(const std::string& from_frame,
                        const rclcpp::Time& stamp,
                        geometry_msgs::msg::TransformStamped& out_tf);
  void scan_to_points_in_target(
      const sensor_msgs::msg::LaserScan& scan,
      const geometry_msgs::msg::TransformStamped& tf,
      std::vector<std::pair<double,double>>& out_xy);
  void process_pair(const sensor_msgs::msg::LaserScan::SharedPtr& s1,
                    const sensor_msgs::msg::LaserScan::SharedPtr& s2);
};

// Utility functions
namespace utils
{
inline bool is_valid_range(float r) {
  return std::isfinite(r) && r > 0.0f;
}
} // namespace utils

} // namespace loader_sim_laserscan_fusion

#endif // LOADER_SIM_LASERSCAN_FUSION__LASER_SCAN_FUSION_NODE_HPP_