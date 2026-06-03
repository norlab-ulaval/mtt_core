#include "mtt_perception/mtt_trailer_estimator_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mtt_perception::TrailerEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
