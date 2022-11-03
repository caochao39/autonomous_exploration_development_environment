#ifndef WAYPOINT_TOOL_H
#define WAYPOINT_TOOL_H

#include <sstream>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <QObject>

#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <rviz_common/display_context.hpp>
#include "rviz_common/properties/string_property.hpp"
#include <rviz_common/tool.hpp>
class rviz_common::properties::StringProperty;
namespace waypoint_rviz_plugin
{
class WaypointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT
public:
  WaypointTool();
  virtual ~WaypointTool()
  {
  }
  virtual void onInitialize();

protected:
  virtual void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  float vehicle_z;

  // ros::NodeHandle nh_;
  // shared_ptr<rclcpp::Node> nh_;
  rclcpp::Node::SharedPtr nh_;
  // ros::Subscriber sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  // ros::Publisher pub_;
  // shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  // ros::Publisher pub_joy_;
  // shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Joy>> pub_joy_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;
  rviz_common::properties::StringProperty* topic_property_;
};
}


#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
