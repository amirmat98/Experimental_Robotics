#include <math.h>

#include <memory>
#include <string>

#include "arucoDetector.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
/**
 * @brief Constructor for the ExploreWaypointAction class.
 *
 * This constructor initializes the ExploreWaypointAction class with the name "explore_waypoint" and a timeout of 1 second.
 *
 * @param name The name of the action.
 * @param timeout The timeout for the action.
 */
ExploreWaypointAction(const std::string& name, const std::chrono::duration<double>& timeout)
  : plansys2::ActionExecutorClient(name, timeout) {}

/**
 * @brief Main function for the ExploreWaypointActionNode.
 *
 * This function initializes the ROS2 environment, creates an instance of the ExploreWaypointAction class, sets the action name parameter, triggers the transition to the CONFIGURE state, spins the node's event loop, and finally shuts down the ROS2 environment.
 *
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments themselves.
 *
 * @return An integer exit code, typically 0 for success.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreWaypointAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "explore_waypoint"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}

