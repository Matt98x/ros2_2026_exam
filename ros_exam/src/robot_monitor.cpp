#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ros_exam/msg/closest_obstacle.hpp"
#include "ros_exam/srv/get_average_velocity.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using ClosestObstacle = ros_exam::msg::ClosestObstacle;
using GetAverageVelocity = ros_exam::srv::GetAverageVelocity;
using GetParameters = rcl_interfaces::srv::GetParameters;

class RobotMonitor : public rclcpp::Node
{
public:
  RobotMonitor() : Node("robot_monitor")
  {
    obstacle_pub_ = create_publisher<ClosestObstacle>("/closest_obstacle", 10);
    scan_sub_ = create_subscription<LaserScan>("/scan", rclcpp::SensorDataQoS(),
      [this](LaserScan::ConstSharedPtr scan) { publish_obstacle(*scan); });
    input_sub_ = create_subscription<Twist>("/cmd_vel", 10,
      [this](Twist::ConstSharedPtr input) {
        if (std::isfinite(input->linear.x) && std::isfinite(input->angular.z)) {
          inputs_.emplace_back(input->linear.x, input->angular.z);
          if (inputs_.size() > 5) {
            inputs_.pop_front();
          }
        }
      });
    average_service_ = create_service<GetAverageVelocity>("/get_average_velocity",
      [this](std::shared_ptr<GetAverageVelocity::Request>,
             std::shared_ptr<GetAverageVelocity::Response> response) {
        response->sample_count = inputs_.size();
        response->success = !inputs_.empty();
        if (inputs_.empty()) {
          return;
        }
        for (const auto & [linear, angular] : inputs_) {
          response->average_linear += linear;
          response->average_angular += angular;
        }
        response->average_linear /= inputs_.size();
        response->average_angular /= inputs_.size();
      });
    threshold_client_ = create_client<GetParameters>("/safe_drive/get_parameters");
    threshold_timer_ = create_wall_timer(std::chrono::milliseconds(500),
      [this]() { update_threshold(); });
  }

private:
  void update_threshold()
  {
    threshold_client_->prune_pending_requests();
    if (!threshold_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<GetParameters::Request>();
    request->names = {"safety_threshold"};
    threshold_client_->async_send_request(request,
      [this](rclcpp::Client<GetParameters>::SharedFuture future) {
        const auto values = future.get()->values;
        if (values.size() == 1 &&
          values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
        {
          threshold_ = values[0].double_value;
        }
      });
  }

  void publish_obstacle(const LaserScan & scan)
  {
    constexpr double pi = 3.14159265358979323846;
    ClosestObstacle result;
    result.header = scan.header;
    result.threshold = threshold_;
    result.distance = std::numeric_limits<double>::infinity();
    result.angle = std::numeric_limits<double>::quiet_NaN();
    result.direction = "unknown";
    bool valid_scan = false;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      if (std::isnan(scan.ranges[i])) {
        continue;
      }
      valid_scan = true;
      // Treat below-minimum readings as close, matching the safety controller.
      const double distance = std::max(0.0, static_cast<double>(scan.ranges[i]));
      if (distance < result.distance) {
        result.distance = distance;
        const double angle = scan.angle_min + i * static_cast<double>(scan.angle_increment);
        result.angle = std::atan2(std::sin(angle), std::cos(angle));
      }
    }
    if (!valid_scan) {
      result.distance = std::numeric_limits<double>::quiet_NaN();
    } else if (std::isinf(result.distance)) {
      result.direction = "none";
    } else if (std::isfinite(result.angle)) {
      if (std::abs(result.angle) <= pi / 4) {
        result.direction = "front";
      } else if (std::abs(result.angle) >= 3 * pi / 4) {
        result.direction = "back";
      } else {
        result.direction = result.angle > 0 ? "left" : "right";
      }
    }
    obstacle_pub_->publish(result);
  }

  double threshold_ = 0.5;
  std::deque<std::pair<double, double>> inputs_;
  rclcpp::Publisher<ClosestObstacle>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Twist>::SharedPtr input_sub_;
  rclcpp::Service<GetAverageVelocity>::SharedPtr average_service_;
  rclcpp::Client<GetParameters>::SharedPtr threshold_client_;
  rclcpp::TimerBase::SharedPtr threshold_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMonitor>());
  rclcpp::shutdown();
  return 0;
}
