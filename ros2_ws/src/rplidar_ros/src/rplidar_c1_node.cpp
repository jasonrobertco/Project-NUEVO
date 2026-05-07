#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sl_lidar.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr std::size_t kMaxScanNodes = 8192;

double deg_to_rad(double degrees)
{
  return degrees * kPi / 180.0;
}

double normalize_angle(double angle)
{
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  return angle;
}

float node_angle_deg(const sl_lidar_response_measurement_node_hq_t & node)
{
  return node.angle_z_q14 * 90.0f / 16384.0f;
}

float node_range_m(const sl_lidar_response_measurement_node_hq_t & node)
{
  return static_cast<float>(node.dist_mm_q2) / 4.0f / 1000.0f;
}

float node_intensity(const sl_lidar_response_measurement_node_hq_t & node)
{
  return static_cast<float>(node.quality >> 2);
}
}  // namespace

class RplidarC1Node : public rclcpp::Node
{
public:
  RplidarC1Node()
  : Node("rplidar_c1")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/rplidar");
    serial_baudrate_ = declare_parameter<int>("serial_baudrate", 460800);
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    topic_name_ = declare_parameter<std::string>("topic_name", "scan");
    scan_mode_ = declare_parameter<std::string>("scan_mode", "Standard");
    scan_frequency_hz_ = declare_parameter<double>("scan_frequency", 10.0);
    range_min_m_ = declare_parameter<double>("range_min", 0.05);
    range_max_m_ = declare_parameter<double>("range_max", 12.0);
    angle_compensate_ = declare_parameter<bool>("angle_compensate", true);
    inverted_ = declare_parameter<bool>("inverted", false);
    flip_x_axis_ = declare_parameter<bool>("flip_x_axis", false);
    reconnect_delay_s_ = declare_parameter<double>("reconnect_delay", 3.0);
    scan_timeout_ms_ = declare_parameter<int>("scan_timeout_ms", 2000);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      topic_name_, rclcpp::SensorDataQoS());

    running_.store(true);
    worker_ = std::thread(&RplidarC1Node::worker_loop, this);

    RCLCPP_INFO(
      get_logger(),
      "RPLIDAR C1 node started: port=%s baud=%d topic=%s frame=%s mode=%s",
      serial_port_.c_str(), serial_baudrate_, topic_name_.c_str(), frame_id_.c_str(),
      scan_mode_.c_str());
  }

  ~RplidarC1Node() override
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    cleanup_driver();
  }

private:
  void worker_loop()
  {
    while (rclcpp::ok() && running_.load()) {
      if (!connect_and_start()) {
        cleanup_driver();
        sleep_for_reconnect_delay();
        continue;
      }

      scan_loop();
      cleanup_driver();
      sleep_for_reconnect_delay();
    }
  }

  bool connect_and_start()
  {
    if (::access(serial_port_.c_str(), F_OK) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR C1 is not connected: device path %s is missing. Waiting for the lidar to be plugged in.",
        serial_port_.c_str());
      return false;
    }

    if (::access(serial_port_.c_str(), R_OK | W_OK) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR C1 was detected at %s, but this process cannot access it (%s). Check udev rules and dialout permissions.",
        serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    auto driver_result = sl::createLidarDriver();
    if (!driver_result || *driver_result == nullptr) {
      RCLCPP_ERROR(get_logger(), "Failed to create Slamtec lidar driver");
      return false;
    }
    driver_.reset(*driver_result);

    auto channel_result = sl::createSerialPortChannel(serial_port_, serial_baudrate_);
    if (!channel_result || *channel_result == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create serial channel for %s", serial_port_.c_str());
      return false;
    }
    channel_.reset(*channel_result);

    sl_result result = driver_->connect(channel_.get());
    if (SL_IS_FAIL(result)) {
      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR C1 was detected at %s, but the device did not respond on the serial connection: 0x%08x",
        serial_port_.c_str(), result);
      return false;
    }

    if (!log_device_info()) {
      return false;
    }
    if (!check_health()) {
      return false;
    }
    if (!start_scan()) {
      return false;
    }

    RCLCPP_INFO(get_logger(), "RPLIDAR C1 is publishing %s", topic_name_.c_str());
    return true;
  }

  bool log_device_info()
  {
    sl_lidar_response_device_info_t info{};
    sl_result result = driver_->getDeviceInfo(info);
    if (SL_IS_FAIL(result)) {
      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR C1 responded on %s, but device info could not be read: 0x%08x",
        serial_port_.c_str(), result);
      return false;
    }

    std::array<char, 33> serial{};
    for (int i = 0; i < 16; ++i) {
      std::snprintf(serial.data() + (i * 2), 3, "%02X", info.serialnum[i]);
    }

    std::string model_name;
    if (SL_IS_FAIL(driver_->getModelNameDescriptionString(model_name, true, &info))) {
      model_name = "unknown";
    }

    RCLCPP_INFO(
      get_logger(),
      "Connected RPLIDAR model=%s raw_model=%u firmware=%u.%02u hardware=%u serial=%s",
      model_name.c_str(), static_cast<unsigned int>(info.model),
      static_cast<unsigned int>(info.firmware_version >> 8),
      static_cast<unsigned int>(info.firmware_version & 0xFF),
      static_cast<unsigned int>(info.hardware_version), serial.data());
    return true;
  }

  bool check_health()
  {
    sl_lidar_response_device_health_t health{};
    sl_result result = driver_->getHealth(health);
    if (SL_IS_FAIL(result)) {
      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR C1 responded on %s, but health status could not be read: 0x%08x",
        serial_port_.c_str(), result);
      return false;
    }

    if (health.status == SL_LIDAR_STATUS_ERROR) {
      RCLCPP_ERROR(
        get_logger(),
        "RPLIDAR reports internal error. Power-cycle the lidar before retrying.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "RPLIDAR health status=%u error_code=%u",
      static_cast<unsigned int>(health.status),
      static_cast<unsigned int>(health.error_code));
    return true;
  }

  bool start_scan()
  {
    sl::LidarScanMode active_mode{};
    sl_result result = SL_RESULT_OPERATION_FAIL;

    if (!scan_mode_.empty()) {
      std::vector<sl::LidarScanMode> modes;
      result = driver_->getAllSupportedScanModes(modes);
      if (SL_IS_FAIL(result)) {
        RCLCPP_WARN(
          get_logger(), "Could not query scan modes; falling back to typical scan: 0x%08x",
          result);
        result = driver_->startScan(false, true, 0, &active_mode);
      } else {
        auto selected = std::find_if(
          modes.begin(), modes.end(),
          [this](const sl::LidarScanMode & mode) {
            return std::string(mode.scan_mode) == scan_mode_;
          });

        if (selected == modes.end()) {
          RCLCPP_ERROR(get_logger(), "RPLIDAR scan mode '%s' is not supported", scan_mode_.c_str());
          for (const auto & mode : modes) {
            RCLCPP_ERROR(
              get_logger(), "supported mode: %s max_distance=%.1fm sample_time=%.2fus",
              mode.scan_mode, mode.max_distance, mode.us_per_sample);
          }
          return false;
        }
        result = driver_->startScanExpress(false, selected->id, 0, &active_mode);
      }
    } else {
      result = driver_->startScan(false, true, 0, &active_mode);
    }

    if (SL_IS_FAIL(result)) {
      RCLCPP_WARN(
        get_logger(),
        "RPLIDAR C1 connected on %s, but scanning could not be started: 0x%08x",
        serial_port_.c_str(), result);
      return false;
    }

    if (active_mode.max_distance > 0.0f) {
      range_max_m_ = active_mode.max_distance;
    }

    if (active_mode.us_per_sample > 0.0f && scan_frequency_hz_ > 0.0) {
      const auto points_per_scan = static_cast<int>(
        1000.0 * 1000.0 / active_mode.us_per_sample / scan_frequency_hz_);
      angle_compensate_multiple_ = std::max(
        1, static_cast<int>(std::ceil(static_cast<double>(points_per_scan) / 360.0)));
    } else {
      angle_compensate_multiple_ = 1;
    }

    const auto rpm = static_cast<sl_u16>(
      std::clamp(scan_frequency_hz_, 1.0, 20.0) * 60.0);
    sl_result motor_result = driver_->setMotorSpeed(rpm);
    if (SL_IS_FAIL(motor_result)) {
      RCLCPP_WARN(get_logger(), "Failed to set RPLIDAR motor speed: 0x%08x", motor_result);
    }

    RCLCPP_INFO(
      get_logger(),
      "RPLIDAR scan active: mode=%s max_distance=%.1fm scan_frequency=%.1fHz bins_per_degree=%d",
      active_mode.scan_mode, range_max_m_, scan_frequency_hz_, angle_compensate_multiple_);
    has_published_scan_ = false;
    consecutive_scan_timeouts_ = 0;
    return true;
  }

  void scan_loop()
  {
    while (rclcpp::ok() && running_.load() && driver_ != nullptr) {
      std::array<sl_lidar_response_measurement_node_hq_t, kMaxScanNodes> nodes{};
      std::size_t count = nodes.size();

      const rclcpp::Time start_time = now();
      sl_result result = driver_->grabScanDataHq(
        nodes.data(), count, static_cast<sl_u32>(std::max(scan_timeout_ms_, 1)));
      const double scan_duration = (now() - start_time).seconds();

      if (result == SL_RESULT_OPERATION_TIMEOUT) {
        ++consecutive_scan_timeouts_;
        if (!has_published_scan_) {
          if (consecutive_scan_timeouts_ == 1) {
            RCLCPP_INFO(
              get_logger(),
              "RPLIDAR C1 is connected and waiting for the first complete scan.");
          } else if (consecutive_scan_timeouts_ % 5 == 0) {
            RCLCPP_WARN(
              get_logger(),
              "RPLIDAR C1 is still waiting for the first complete scan after %d consecutive timeouts.",
              consecutive_scan_timeouts_);
          }
        } else if (consecutive_scan_timeouts_ == 3 || consecutive_scan_timeouts_ % 10 == 0) {
          RCLCPP_WARN(
            get_logger(),
            "RPLIDAR scan timed out %d times in a row after scanning had already started. The device may have stopped responding.",
            consecutive_scan_timeouts_);
        }
        continue;
      }

      if (SL_IS_FAIL(result)) {
        RCLCPP_WARN(get_logger(), "RPLIDAR scan read failed: 0x%08x", result);
        break;
      }

      result = driver_->ascendScanData(nodes.data(), count);
      if (SL_IS_FAIL(result)) {
        RCLCPP_WARN(get_logger(), "RPLIDAR scan sort failed: 0x%08x", result);
        continue;
      }

      if (count > 1) {
        if (!has_published_scan_ && consecutive_scan_timeouts_ > 0) {
          RCLCPP_INFO(
            get_logger(),
            "RPLIDAR C1 received its first complete scan after %d startup timeout(s).",
            consecutive_scan_timeouts_);
        }
        has_published_scan_ = true;
        consecutive_scan_timeouts_ = 0;
        publish_scan(nodes.data(), count, start_time, scan_duration);
      }
    }
  }

  void publish_scan(
    const sl_lidar_response_measurement_node_hq_t * nodes,
    std::size_t count,
    const rclcpp::Time & stamp,
    double scan_duration)
  {
    auto msg = sensor_msgs::msg::LaserScan();
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.range_min = static_cast<float>(range_min_m_);
    msg.range_max = static_cast<float>(range_max_m_);
    msg.scan_time = static_cast<float>(scan_duration);

    if (angle_compensate_) {
      publish_compensated_scan(msg, nodes, count);
    } else {
      publish_raw_ordered_scan(msg, nodes, count);
    }
  }

  void publish_compensated_scan(
    sensor_msgs::msg::LaserScan & msg,
    const sl_lidar_response_measurement_node_hq_t * nodes,
    std::size_t count)
  {
    const std::size_t bins = static_cast<std::size_t>(360 * angle_compensate_multiple_);
    msg.angle_min = static_cast<float>(-kPi);
    msg.angle_max = static_cast<float>(kPi);
    msg.angle_increment = static_cast<float>((msg.angle_max - msg.angle_min) / (bins - 1));
    msg.time_increment = msg.scan_time / static_cast<float>(std::max<std::size_t>(bins - 1, 1));
    msg.ranges.assign(bins, std::numeric_limits<float>::infinity());
    msg.intensities.assign(bins, 0.0f);

    for (std::size_t i = 0; i < count; ++i) {
      const float range = node_range_m(nodes[i]);
      if (range < msg.range_min || range > msg.range_max) {
        continue;
      }

      const double ros_angle = ros_angle_from_lidar_angle(node_angle_deg(nodes[i]));
      const auto index = static_cast<std::size_t>(std::clamp(
        static_cast<long>(std::lround((ros_angle - msg.angle_min) / msg.angle_increment)),
        0L,
        static_cast<long>(bins - 1)));

      if (!std::isfinite(msg.ranges[index]) || range < msg.ranges[index]) {
        msg.ranges[index] = range;
        msg.intensities[index] = node_intensity(nodes[i]);
      }
    }

    scan_pub_->publish(msg);
  }

  void publish_raw_ordered_scan(
    sensor_msgs::msg::LaserScan & msg,
    const sl_lidar_response_measurement_node_hq_t * nodes,
    std::size_t count)
  {
    struct Point
    {
      double angle;
      float range;
      float intensity;
    };

    std::vector<Point> points;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const float range = node_range_m(nodes[i]);
      if (range < msg.range_min || range > msg.range_max) {
        continue;
      }
      points.push_back({
        ros_angle_from_lidar_angle(node_angle_deg(nodes[i])),
        range,
        node_intensity(nodes[i]),
      });
    }

    if (points.size() < 2) {
      return;
    }

    std::sort(points.begin(), points.end(), [](const Point & a, const Point & b) {
      return a.angle < b.angle;
    });

    msg.angle_min = static_cast<float>(points.front().angle);
    msg.angle_max = static_cast<float>(points.back().angle);
    msg.angle_increment = static_cast<float>(
      (msg.angle_max - msg.angle_min) / static_cast<double>(points.size() - 1));
    msg.time_increment = msg.scan_time / static_cast<float>(points.size() - 1);
    msg.ranges.resize(points.size());
    msg.intensities.resize(points.size());

    for (std::size_t i = 0; i < points.size(); ++i) {
      msg.ranges[i] = points[i].range;
      msg.intensities[i] = points[i].intensity;
    }

    scan_pub_->publish(msg);
  }

  double ros_angle_from_lidar_angle(double lidar_angle_deg) const
  {
    double lidar_angle = deg_to_rad(lidar_angle_deg);
    if (inverted_) {
      lidar_angle = -lidar_angle;
    }

    double ros_angle = normalize_angle(kPi - lidar_angle);
    if (flip_x_axis_) {
      ros_angle = normalize_angle(ros_angle + kPi);
    }
    return ros_angle;
  }

  void cleanup_driver()
  {
    if (driver_ != nullptr) {
      driver_->stop();
      driver_->setMotorSpeed(0);
      driver_->disconnect();
      driver_.reset();
    }
    channel_.reset();
  }

  void sleep_for_reconnect_delay() const
  {
    const auto delay = std::chrono::duration<double>(std::max(reconnect_delay_s_, 0.1));
    const auto deadline = std::chrono::steady_clock::now() + delay;
    while (running_.load() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  std::string serial_port_;
  int serial_baudrate_;
  std::string frame_id_;
  std::string topic_name_;
  std::string scan_mode_;
  double scan_frequency_hz_;
  double range_min_m_;
  double range_max_m_;
  bool angle_compensate_;
  bool inverted_;
  bool flip_x_axis_;
  double reconnect_delay_s_;
  int scan_timeout_ms_;
  int angle_compensate_multiple_{1};
  bool has_published_scan_{false};
  int consecutive_scan_timeouts_{0};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::unique_ptr<sl::ILidarDriver> driver_;
  std::unique_ptr<sl::IChannel> channel_;
  std::atomic_bool running_{false};
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RplidarC1Node>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
