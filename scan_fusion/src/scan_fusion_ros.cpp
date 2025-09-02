#include "loader_sim_laserscan_fusion/scan_fusion_ros.hpp"

using std::placeholders::_1;

namespace loader_sim_laserscan_fusion
{

LaserScanFusionNode::LaserScanFusionNode(const rclcpp::NodeOptions & options)
: Node("laserscan_fusion_node", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Parameters
  scan1_topic_ = this->declare_parameter<std::string>("scan1_topic", "/scan1");
  scan2_topic_ = this->declare_parameter<std::string>("scan2_topic", "/scan2");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/scan");
  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_footprint");

  angle_min_ = this->declare_parameter<double>("grid_angle_min", -M_PI);
  angle_max_ = this->declare_parameter<double>("grid_angle_max", M_PI);
  angle_inc_ = this->declare_parameter<double>("grid_angle_increment", 0.004363323129985824); // 0.25 deg

  out_range_min_ = this->declare_parameter<double>("range_min", 0.05);
  out_range_max_ = this->declare_parameter<double>("range_max", 30.0);

  fusion_mode_ = this->declare_parameter<std::string>("fusion_mode", "min"); // "min" or "weighted"
  weight_scan1_ = this->declare_parameter<double>("weight_scan1", 1.0);
  weight_scan2_ = this->declare_parameter<double>("weight_scan2", 0.5); // 伪激光默认权重小

  use_inf_ = this->declare_parameter<bool>("use_inf_for_no_data", true);
  fill_intensities_ = this->declare_parameter<bool>("fill_intensities", false);

  queue_size_ = this->declare_parameter<int>("queue_size", 10);
  slop_sec_ = this->declare_parameter<double>("slop", 0.05); // 50 ms

  tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout", 0.05);

  // Derived
  if (angle_max_ <= angle_min_) {
    RCLCPP_FATAL(this->get_logger(), "grid_angle_max must be > grid_angle_min");
    throw std::runtime_error("Invalid angle range");
  }
  if (angle_inc_ <= 0.0) {
    RCLCPP_FATAL(this->get_logger(), "grid_angle_increment must be > 0");
    throw std::runtime_error("Invalid angle increment");
  }

  // Publisher
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, rclcpp::SensorDataQoS());

  // Subscriptions
  rclcpp::SubscriptionOptions options_sub;
  sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan1_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LaserScanFusionNode::scan1_cb, this, _1), options_sub);

  sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan2_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LaserScanFusionNode::scan2_cb, this, _1), options_sub);

  RCLCPP_INFO(this->get_logger(), "laserscan_fusion started. scan1=%s, scan2=%s, out=%s, target_frame=%s",
              scan1_topic_.c_str(), scan2_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
}

void LaserScanFusionNode::scan1_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  push_and_trim(buf1_, msg);
  try_match_and_process();
}

void LaserScanFusionNode::scan2_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  push_and_trim(buf2_, msg);
  try_match_and_process();
}

void LaserScanFusionNode::push_and_trim(std::deque<sensor_msgs::msg::LaserScan::SharedPtr>& buf,
                                        const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
  buf.push_back(msg);
  while (static_cast<int>(buf.size()) > queue_size_) {
    buf.pop_front();
  }
}

void LaserScanFusionNode::try_match_and_process()
{
  if (buf1_.empty() || buf2_.empty()) {
    return;
  }
  // Find nearest timestamps
  auto best_pair = find_best_pair();
  if (best_pair.first && best_pair.second) {
    auto t1 = best_pair.first->header.stamp;
    auto t2 = best_pair.second->header.stamp;
    const double dt = std::fabs((rclcpp::Time(t1) - rclcpp::Time(t2)).seconds());
    if (dt <= slop_sec_) {
      // Remove from buffers
      process_pair(best_pair.first, best_pair.second);
      erase_msg(buf1_, best_pair.first);
      erase_msg(buf2_, best_pair.second);
    } else {
      // Too far apart; drop older to keep moving
      if (rclcpp::Time(buf1_.front()->header.stamp) < rclcpp::Time(buf2_.front()->header.stamp)) {
        buf1_.pop_front();
      } else {
        buf2_.pop_front();
      }
    }
  }
}

void LaserScanFusionNode::erase_msg(std::deque<sensor_msgs::msg::LaserScan::SharedPtr>& buf,
                                    const sensor_msgs::msg::LaserScan::SharedPtr& target)
{
  for (auto it = buf.begin(); it != buf.end(); ++it) {
    if ((*it).get() == target.get()) {
      buf.erase(it);
      return;
    }
  }
}

std::pair<sensor_msgs::msg::LaserScan::SharedPtr, sensor_msgs::msg::LaserScan::SharedPtr>
LaserScanFusionNode::find_best_pair()
{
  sensor_msgs::msg::LaserScan::SharedPtr a_best, b_best;
  double best_dt = std::numeric_limits<double>::infinity();
  for (auto& a : buf1_) {
    for (auto& b : buf2_) {
      double dt = std::fabs((rclcpp::Time(a->header.stamp) - rclcpp::Time(b->header.stamp)).seconds());
      if (dt < best_dt) {
        best_dt = dt;
        a_best = a;
        b_best = b;
      }
    }
  }
  return {a_best, b_best};
}

bool LaserScanFusionNode::lookup_transform(const std::string& from_frame,
                                           const rclcpp::Time& stamp,
                                           geometry_msgs::msg::TransformStamped& out_tf)
{
  // RCLCPP_WARN(this->get_logger(), "from_frame is %s, to_frame is %s", 
  // from_frame.c_str(), target_frame_.c_str());
  try {
    out_tf = tf_buffer_.lookupTransform(
      target_frame_, from_frame, stamp,
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "TF lookup failed from %s to %s: %s",
                         from_frame.c_str(), target_frame_.c_str(), ex.what());
    return false;
  }
}

void LaserScanFusionNode::scan_to_points_in_target(
    const sensor_msgs::msg::LaserScan& scan,
    const geometry_msgs::msg::TransformStamped& tf,
    std::vector<std::pair<double,double>>& out_xy)
{
  out_xy.clear();
  out_xy.reserve(scan.ranges.size());
  double angle = scan.angle_min;
  for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
    const float r = scan.ranges[i];
    if (!utils::is_valid_range(r)) continue;
    if (r < scan.range_min || r > scan.range_max) continue;

    // Point in scanner frame
    geometry_msgs::msg::PointStamped ps_in, ps_out;
    ps_in.header = scan.header;
    ps_in.point.x = static_cast<double>(r) * std::cos(angle);
    ps_in.point.y = static_cast<double>(r) * std::sin(angle);
    ps_in.point.z = 0.0;

    tf2::doTransform(ps_in, ps_out, tf);

    out_xy.emplace_back(ps_out.point.x, ps_out.point.y);
  }
}

void LaserScanFusionNode::process_pair(const sensor_msgs::msg::LaserScan::SharedPtr& s1,
                                       const sensor_msgs::msg::LaserScan::SharedPtr& s2)
{
  // TF to target frame for both
  geometry_msgs::msg::TransformStamped tf1, tf2;
  if (!lookup_transform(s1->header.frame_id, rclcpp::Time(s1->header.stamp), tf1)) return;
  if (!lookup_transform(s2->header.frame_id, rclcpp::Time(s2->header.stamp), tf2)) return;

  // Convert to points in target frame
  std::vector<std::pair<double,double>> pts1, pts2;
  scan_to_points_in_target(*s1, tf1, pts1);
  scan_to_points_in_target(*s2, tf2, pts2);

  // Allocate output arrays
  const int n = static_cast<int>(std::floor((angle_max_ - angle_min_) / angle_inc_)) + 1;
  std::vector<float> r1bins(n, std::numeric_limits<float>::infinity());
  std::vector<float> r2bins(n, std::numeric_limits<float>::infinity());

  auto bin_point = [&](const std::pair<double,double>& xy, std::vector<float>& bins) {
    const double phi = std::atan2(xy.second, xy.first);
    const double rho = std::hypot(xy.first, xy.second);
    if (phi < angle_min_ || phi > angle_max_) return;
    int idx = static_cast<int>(std::round((phi - angle_min_) / angle_inc_));
    if (idx < 0 || idx >= n) return;
    float cur = bins[idx];
    if (rho < cur) {
      bins[idx] = static_cast<float>(rho);
    }
  };

  for (const auto& p : pts1) bin_point(p, r1bins);
  for (const auto& p : pts2) bin_point(p, r2bins);

  // Fuse
  sensor_msgs::msg::LaserScan out;
  out.header.stamp = rclcpp::Time(s1->header.stamp) < rclcpp::Time(s2->header.stamp) ? s2->header.stamp : s1->header.stamp;
  out.header.frame_id = target_frame_;
  out.angle_min = static_cast<float>(angle_min_);
  out.angle_max = static_cast<float>(angle_max_);
  out.angle_increment = static_cast<float>(angle_inc_);
  out.time_increment = 0.0f; // unknown
  out.scan_time = 0.0f; // unknown
  out.range_min = static_cast<float>(out_range_min_);
  out.range_max = static_cast<float>(out_range_max_);
  out.ranges.resize(n, use_inf_ ? std::numeric_limits<float>::infinity() : static_cast<float>(out_range_max_));
  if (fill_intensities_) out.intensities.resize(n, 0.0f);

  const bool is_min = (fusion_mode_ == "min");
  const double w1 = weight_scan1_;
  const double w2 = weight_scan2_;

  for (int i = 0; i < n; ++i) {
    const float a = r1bins[i];
    const float b = r2bins[i];
    bool a_ok = std::isfinite(a);
    bool b_ok = std::isfinite(b);
    float fused = use_inf_ ? std::numeric_limits<float>::infinity() : static_cast<float>(out_range_max_);
    
    // 使用最小值
    if (is_min) {
      if (a_ok && b_ok) fused = std::min(a, b);
      else if (a_ok) fused = a;
      else if (b_ok) fused = b;
    } else {
      // weighted
      if (a_ok && b_ok) {
        fused = static_cast<float>((w1 * a + w2 * b) / (w1 + w2));
      } else if (a_ok) {
        fused = a;
      } else if (b_ok) {
        fused = b;
      }
    }
    
    // clamp to output range
    if (std::isfinite(fused)) {
      if (fused < out.range_min) fused = out.range_min;
      if (fused > out.range_max) fused = use_inf_ ? std::numeric_limits<float>::infinity() : out.range_max;
    }
    out.ranges[i] = fused;

    if (fill_intensities_) {
      // 1:来自scan1, 2:来自scan2, 3:两者（min/weighted）
      float src = 0.0f;
      if (is_min) {
        if (a_ok && b_ok) {
          src = (a <= b) ? 1.0f : 2.0f;
        } else if (a_ok) src = 1.0f;
        else if (b_ok) src = 2.0f;
        else src = 0.0f;
      } else {
        if (a_ok && b_ok) src = 3.0f;
        else if (a_ok) src = 1.0f;
        else if (b_ok) src = 2.0f;
        else src = 0.0f;
      }
      out.intensities[i] = src;
    }
  }

  pub_->publish(out);
}

} // namespace loader_sim_laserscan_fusion