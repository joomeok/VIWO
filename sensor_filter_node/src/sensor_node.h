#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <string>
#include <sensor_msgs/Imu.h>

class sensor_filtering_node
{
public:
  sensor_filtering_node(ros::NodeHandle nh){
    // subscribe to the topics
    vo_sub = nh.subscribe("/orb_slam2_stereo/pose", 1, &sensor_filtering_node::voCallback, this);
    range_sub = nh.subscribe("/front_range", 1 ,&sensor_filtering_node::rangeCallback, this);
    pipe_sub = nh.subscribe("/on_rails", 1, &sensor_filtering_node::pipeCallback, this);
    ekf_sub = nh.subscribe("/odom/vil", 1, &sensor_filtering_node::ekfCallback, this);

    // publish to the topics
    on_headland_odom_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom/on_headland", 1);
    on_rails_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom/on_rails", 1);

    // fix the covariance of rf
    for (int i = 0; i < 6; i++) {
      on_rails_pose.pose.covariance[7 * i] = 0.1; // last 0.1
    }
  }

  void pipeCallback(const std_msgs::Bool::ConstPtr &msg){
    if (is_on_rails != msg->data) {
      if (msg->data) ROS_INFO("On the rails");
      else ROS_INFO("on the headland");
    }

    is_on_rails = msg->data;
  }

  void voCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
    if (!is_on_rails) {
      if (last_status){
        // calculate offset
        offset_pose.position.x = ekf_pose.position.x - msg->pose.pose.position.x;
        offset_pose.position.y = ekf_pose.position.y - msg->pose.pose.position.y;
        offset_pose.position.z = ekf_pose.position.z - msg->pose.pose.position.z;

        offset_pose.orientation.x = ekf_pose.orientation.x - msg->pose.pose.orientation.x;
        offset_pose.orientation.y = ekf_pose.orientation.y - msg->pose.pose.orientation.y;
        offset_pose.orientation.z = ekf_pose.orientation.z - msg->pose.pose.orientation.z;
        offset_pose.orientation.w = ekf_pose.orientation.w - msg->pose.pose.orientation.w;

        ROS_INFO("updated offset >> %f %f", offset_pose.position.x, offset_pose.position.y);
        last_status = false;
      }

      on_headland_pose.header.frame_id = msg->header.frame_id;
      on_headland_pose.header.stamp = ros::Time::now();
      on_headland_pose.pose.covariance = msg->pose.covariance;
      on_headland_pose.pose.pose.position.x = msg->pose.pose.position.x + offset_pose.position.x;
      on_headland_pose.pose.pose.position.y = msg->pose.pose.position.y + offset_pose.position.y;
      on_headland_pose.pose.pose.position.z = msg->pose.pose.position.z + offset_pose.position.z;
      on_headland_pose.pose.pose.orientation.x = msg->pose.pose.orientation.x + offset_pose.orientation.x;
      on_headland_pose.pose.pose.orientation.y = msg->pose.pose.orientation.y + offset_pose.orientation.y;
      on_headland_pose.pose.pose.orientation.z = msg->pose.pose.orientation.z + offset_pose.orientation.z;
      on_headland_pose.pose.pose.orientation.w = msg->pose.pose.orientation.w + offset_pose.orientation.w;

      on_headland_odom_pub.publish(on_headland_pose);
    }
  }

  void rangeCallback(const sensor_msgs::Range::ConstPtr &msg){
    // range = msg->range;
    if (is_on_rails) {
      if (!last_status) { // save last pose from vo
        ROS_INFO("Set last ekf pose!");
        last_pose = ekf_pose;
        last_status = true;
      }
      on_rails_pose.header.frame_id = "gt_odom";
      on_rails_pose.header.stamp = ros::Time::now();
      filtered_y = wall_pose - msg->range - base_to_rf;

      if (ekf_pose.position.y < 0)
        filtered_y *= -1;

      if (debug) ROS_INFO("filtered_y: %f", filtered_y);

      on_rails_pose.pose.pose = last_pose;
      on_rails_pose.pose.pose.position.y = filtered_y;

      on_rails_pose_pub.publish(on_rails_pose);
    }
  }

  void ekfCallback(const nav_msgs::Odometry::ConstPtr &msg){
    // update latest ekf pose
    ekf_pose = msg->pose.pose;
  }

private:
  // topic publishers
  ros::Publisher on_headland_odom_pub; // from vo
  ros::Publisher on_rails_pose_pub; // from range

  // topic subscribers
  ros::Subscriber pipe_sub;
  ros::Subscriber vo_sub;
  ros::Subscriber range_sub;
  ros::Subscriber ekf_sub;

  bool is_on_rails = false;
  bool last_status = false;
  double filtered_y = 0.0;

  geometry_msgs::Pose last_pose;
  geometry_msgs::Pose offset_pose;
  geometry_msgs::Pose ekf_pose;

  // ros parameters
  bool debug = ros::param::param<bool>("~debug", false);
  double base_to_rf = ros::param::param<float>("~base_to_rf", 0.0);
  double wall_pose = ros::param::param<float>("~wall_pose", 0.0);

  // from vo callback
  geometry_msgs::PoseWithCovarianceStamped on_headland_pose;
  geometry_msgs::PoseWithCovarianceStamped on_rails_pose;
};


