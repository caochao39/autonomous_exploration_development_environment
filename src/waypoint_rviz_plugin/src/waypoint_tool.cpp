#include <waypoint_rviz_plugin/waypoint_tool.h>
// #include <waypoint_tool.h>

namespace waypoint_rviz_plugin
{
WaypointTool::WaypointTool()
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void WaypointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  setName("Waypoint");
  updateTopic();
  vehicle_z = 0;
}

void WaypointTool::updateTopic()
{
  // sub_ = nh_.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, &WaypointTool::odomHandler, this);
  sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5 ,std::bind(&WaypointTool::odomHandler,this,std::placeholders::_1));
  // pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);
  // pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
  pub_joy_ = nh_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
}

void WaypointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  vehicle_z = odom->pose.pose.position.z;
}

void WaypointTool::onPoseSet(double x, double y, double theta)
{
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  // joy.header.stamp = ros::Time::now();
  joy.header.stamp = nh_->now();
  joy.header.frame_id = "waypoint_tool";
  pub_joy_->publish(joy);

  geometry_msgs::msg::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = joy.header.stamp;
  waypoint.point.x = x;
  waypoint.point.y = y;
  waypoint.point.z = vehicle_z;

  pub_->publish(waypoint);
  usleep(10000);
  pub_->publish(waypoint);
}
}

#include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool)
PLUGINLIB_EXPORT_CLASS(waypoint_rviz_plugin::WaypointTool, rviz_default_plugins::tools::PoseTool)
