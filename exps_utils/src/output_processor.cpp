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
#include "mh_amcl_msgs/msg/info.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>

using std::placeholders::_1;
double desviation_roll = 1987.0, desviation_pitch = 1987.0, desviation_yaw = 1987.0;
// clock_t time_init = clock();
double timestamp_in_seconds_init = 1987.0;
int tiago = 1; // 0: Summit, 1: Tiago
int mh_amcl = 1; // 0: AMCL, 1: MH-AMCL

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

    sub_info_ = create_subscription<mh_amcl_msgs::msg::Info>(
      "info", 100, [this](mh_amcl_msgs::msg::Info::UniquePtr msg) {
        this->last_info_ = std::move(msg);
        update();
      });

    if ((file = fopen(filename, "a") ) == NULL) { 
      std::cout << "Error al abrir el fichero" << std::endl;
    }

    if (mh_amcl == 0) {
      fprintf(file,"time,error_translation,error_roll,error_pitch,error_yaw\n");
    } else {
      fprintf(file,"time,error_translation,error_roll,error_pitch,error_yaw,quality,uncertainty,predict_time,correct_time,reseed_time\n");
    }
    
    fclose(file);
  }

private:
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<mocap_msgs::msg::RigidBodies>::SharedPtr sub_gt_;
  rclcpp::Subscription<mh_amcl_msgs::msg::Info>::SharedPtr sub_info_;

  geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr last_pose_;
  mocap_msgs::msg::RigidBodies::UniquePtr last_gt_;
  mh_amcl_msgs::msg::Info::UniquePtr last_info_;

  FILE* file;
  const char* filename = "data.csv";

  void update()
  {
    geometry_msgs::msg::TransformStamped map2bf_msg;
    std::string error;
    if (tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &error)) {
      map2bf_msg = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);

      if (last_gt_ != nullptr && !last_gt_->rigidbodies.empty()/* && last_info_ != nullptr*/) {

        
        
        double timestamp_in_seconds = (rclcpp::Time(last_gt_->header.stamp).seconds()+rclcpp::Time(last_gt_->header.stamp).nanoseconds())/1000000000;

        // Imprimir el resultado
        // std::cout << "Timestamp en sec: " << timestamp_in_seconds-timestamp_in_seconds_init << std::endl;

        // clock_t time_req = clock() - time_init;
        
        // Correction robot initial pose and optitrack axis
        // const auto & gt_x = last_gt_->rigidbodies[0].pose.position.x; 
        // const auto & gt_y = last_gt_->rigidbodies[0].pose.position.y;
        auto gt_x = last_gt_->rigidbodies[0].pose.position.x; // Summit sim
        auto gt_y = last_gt_->rigidbodies[0].pose.position.y; // Summit sim
        if (tiago == 1) {
          gt_x = (-last_gt_->rigidbodies[0].pose.position.y+0.9); // Tiago real
          gt_y = (last_gt_->rigidbodies[0].pose.position.x+0.2); // Tiago real
        }
        
        const auto & robot_x = map2bf_msg.transform.translation.x;
        const auto & robot_y = map2bf_msg.transform.translation.y;

        double error_translation = sqrt((gt_x - robot_x) * (gt_x - robot_x) + (gt_y - robot_y) * (gt_y - robot_y));
        
        tf2::Quaternion q1, q2;
        tf2::fromMsg(last_gt_->rigidbodies[0].pose.orientation , q1);
        tf2::fromMsg(map2bf_msg.transform.rotation , q2);

        tf2::Quaternion q_diff = q2 * q1.inverse();

        double error_roll, error_pitch, error_yaw;
        tf2::Matrix3x3 m_diff(q_diff);
        m_diff.getRPY(error_roll, error_pitch, error_yaw);

        if (desviation_pitch > 1000)
        {
          desviation_pitch = error_pitch;
        }
        if (desviation_roll > 1000)
        {
          desviation_roll = error_roll;
        }
        if (desviation_yaw > 1000)
        {
          desviation_yaw = error_yaw;
        }
        if (timestamp_in_seconds_init == 1987.0)
        {
          timestamp_in_seconds_init = (rclcpp::Time(last_gt_->header.stamp).seconds()+rclcpp::Time(last_gt_->header.stamp).nanoseconds())/1000000000;
        }

        error_roll = std::abs(error_roll-desviation_roll);
        error_pitch = std::abs(error_pitch-desviation_pitch);
        error_yaw = std::abs(error_yaw-desviation_yaw);       

        float timer = timestamp_in_seconds-timestamp_in_seconds_init;
        std::cout << timer << " ";
        std::cout << "- T:" << error_translation << " - R:" << error_roll << " - P:" << error_pitch << " - Y:" << error_yaw << std::endl;
        
        if ((file = fopen(filename, "a") ) == NULL) { 
          std::cout << "Error al abrir el fichero" << std::endl;
        }

        if (last_info_ != nullptr){
          double quality = last_info_->quality;
          double uncertainty = last_info_->uncertainty;
          double predict_time = static_cast<double>(last_info_->predict_time.sec) + static_cast<double>(last_info_->predict_time.nanosec) / 1e9;
          double correct_time = static_cast<double>(last_info_->correct_time.sec) + static_cast<double>(last_info_->correct_time.nanosec) / 1e9;
          double reseed_time = static_cast<double>(last_info_->reseed_time.sec) + static_cast<double>(last_info_->reseed_time.nanosec) / 1e9;

          std::cout << quality << " " << uncertainty << " " << std::endl;
          std::cout << "Time: " << predict_time << " " << correct_time << " " << reseed_time << " " << std::endl;

          if (abs(error_roll) < 0.2 && abs(error_pitch) < 0.2 && abs(error_yaw) < 0.2) {
            fprintf(file,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",timer,error_translation,error_roll,error_pitch,error_yaw,quality,uncertainty,predict_time,correct_time,reseed_time);  
          }
        }else{
          if (abs(error_roll) < 0.2 && abs(error_pitch) < 0.2 && abs(error_yaw) < 0.2) {
            fprintf(file,"%f,%f,%f,%f,%f\n",timer,error_translation,error_roll,error_pitch,error_yaw);  
          }
        }
         
        fclose(file);
      } else {
        std::cout << "No gt or info" << std::endl;
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
