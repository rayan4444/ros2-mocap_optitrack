#include <WorldToBase.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <stdio.h>

#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
// #include "geometry_msgs/msg/PoseWithCovarianceStamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// this node listens to rigid bodies messages and strips them down into just the
// pose message so we can see them in Rviz

using namespace Eigen;
using namespace std;

WorldToBase::WorldToBase(): Node("world_to_base")
{
  this->declare_parameter<std::string>("sub_topic", "rigid_body_topic");
  this->declare_parameter<std::string>("pub_topic", "optitrack_bodies");
  // //
  // Subscribe to the topic for RigidBody messages
  std::string sub_topic_;
  this->get_parameter("sub_topic", sub_topic_);
  char* sub_topic = (char*) malloc(sub_topic_.length()*sizeof(char));
  strcpy(sub_topic, sub_topic_.c_str());
  this->subscription_ = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(
    sub_topic, 10, std::bind(&WorldToBase::rigid_body_topic_callback, this, _1));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  // Log info about creation
  RCLCPP_INFO(this->get_logger(), "Created world to base node.\n");
}

// Callback to receive rigid body messages
void WorldToBase::rigid_body_topic_callback(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  // Transform the pose of all the rigid bodies from the frame of the motion capture system to the base frame of the robot
  transformPoseAndSend(msg);
}

// Method that transforms the pose of the rigid bodies expressed in the motion capture system into the base frame of the robot
void WorldToBase::transformPoseAndSend(const mocap_optitrack_interfaces::msg::RigidBodyArray::SharedPtr msg) const
{
  int i = -1;//i is the iterator, i_base is the index position in the RigidBodyArray
  int nRB = (int) msg->rigid_bodies.size();

  /* For each rigid body get rid of the uless part of the frame and justs give me the pose message */
  for (i = 0; i < nRB; i++)
  {
      // Print some information
      RCLCPP_DEBUG(this->get_logger(), "ID : %ld\n", msg->rigid_bodies[i].id);
      RCLCPP_DEBUG(this->get_logger(), "Time stamp : %d(s)---%d(ns)\n", msg->rigid_bodies[i].pose_stamped.header.stamp.sec, msg->rigid_bodies[i].pose_stamped.header.stamp.nanosec);

      /* Save the tranformed pose in the new message*/
      geometry_msgs::msg::TransformStamped tagtf;

      // Store the position
      // tagtf.header.stamp = this->get_clock()->now();
      tagtf.header.frame_id = "world";
      tagtf.child_frame_id = msg->rigid_bodies[i].pose_stamped.header.frame_id;
      tagtf.transform.translation.x = msg->rigid_bodies[i].pose_stamped.pose.position.x;
      tagtf.transform.translation.y = msg->rigid_bodies[i].pose_stamped.pose.position.y;
      tagtf.transform.translation.z = msg->rigid_bodies[i].pose_stamped.pose.position.z;
      // Sntation through the unit quaternion
      tagtf.transform.rotation.x = msg->rigid_bodies[i].pose_stamped.pose.orientation.x,
      tagtf.transform.rotation.y = msg->rigid_bodies[i].pose_stamped.pose.orientation.y,
      tagtf.transform.rotation.z = msg->rigid_bodies[i].pose_stamped.pose.orientation.z,
      tagtf.transform.rotation.w = msg->rigid_bodies[i].pose_stamped.pose.orientation.w;

      // Publish the message
      // this->publisher_->publish(tagtf);

      tf_broadcaster_->sendTransform(tagtf);

  }
}

// Main method
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WorldToBase>());
    rclcpp::shutdown();
    return 0;
}