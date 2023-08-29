// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mocap_msgs/msg/rigid_bodies.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;

class OutputProcessor : public rclcpp::Node
{
public:
  OutputProcessor()
  : Node("output_processor"),
    tf_buffer_(),
    tf_listener_(tf_buffer_)
  {
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", 100, [this](geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr msg) {
        this->last_pose_ = std::move(msg);
        update();
      });

    sub_gt_ = create_subscription<mocap_msgs::msg::RigidBodies>(
      "rigid_bodies", 100, [this](mocap_msgs::msg::RigidBodies::UniquePtr msg) {
        this->last_gt_ = std::move(msg);
        update();
      });
  }

private:
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr sub_gt_;

  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr last_pose_;
  mocap_msgs::msg::RigidBodies::UniquePtr last_gt_;

  void update()
  {
    geometry_msgs::msg::TransformStamped map2bf_msg;
    std::string error;
    if (tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &error)) {
      map2bf_msg = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);

      if (last_gt_ != nullptr && !last_gt_->rigidbodies.empty()) {
        const auto & gt_x = last_gt_->rigidbodies[0].pose.position.x;
        const auto & gt_y = last_gt_->rigidbodies[0].pose.position.y;
        const auto & robot_x = map2bf_msg.transform.translation.x;
        const auto & robot_y = map2bf_msg.transform.translation.y;


        double error_translation = sqrt((gt_x - robot_x) * (gt_x - robot_x) + (gt_y - robot_y) * (gt_y - robot_y));
        
        tf2::Quaternion q1, q2;
        tf2::fromMsg(last_gt_->rigidbodies[0].pose.orientation , q1);
        tf2::fromMsg(map2bf_msg.transform.rotation , q2);

        tf2::Quaternion q_diff = q2 * q1.inverse();

        double roll, pitch, yaw;
        tf2::Matrix3x3 m_diff(q_diff);
        m_diff.getRPY(roll, pitch, yaw);
        
        std::cout << error_translation << " " << roll << " " << pitch << " " << yaw << std::endl;
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto processor = std::make_shared<OutputProcessor>();
  
  rclcpp::spin(processor);
  rclcpp::shutdown();

  return 0;
}
