#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <xpp_states/convert.h>
#include <towr_ros/TowrCommand.h>
#include <towr_ros/topic_names.h>

namespace towr {

static ros::Publisher rviz_pub;

void UserCommandCallback(const towr_ros::TowrCommand& msg_in)
{
  if (msg_in.handle_lin.size() != msg_in.handle_ang.size()) {
    ROS_ERROR("handle_lin and handle_ang do not match!");
    return;
  }

  // Create marker array message
  visualization_msgs::MarkerArray markers_msg;
  const size_t n_handles = msg_in.handle_lin.size(); 
  markers_msg.markers.resize(msg_in.handle_lin.size()); // One marker per handle

  for (size_t i = 0; i < n_handles; ++i) {
    auto& m = markers_msg.markers[i];
    m.header.frame_id = "world";
    m.header.stamp = ros::Time();
    m.ns = "handles";
    m.id = i;
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position = msg_in.handle_lin[i].pos;
    Eigen::Quaterniond q = xpp::GetQuaternionFromEulerZYX(msg_in.handle_ang[i].pos.z,
							  msg_in.handle_ang[i].pos.y,
							  msg_in.handle_ang[i].pos.z);
    m.pose.orientation = xpp::Convert::ToRos(q);
    m.scale.x = 1; m.scale.y = 1; m.scale.z = 1;
    m.color.a = 1.0;
    m.color.r = 0.7;
    m.color.g = 0.7;
    m.color.b = 0.7;
    m.mesh_resource = "package://xpp_talos/meshes/handle.dae";
  }

  rviz_pub.publish(markers_msg);
}

} // namespace towr

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rviz_handles_visualizer");

  ros::NodeHandle n;

  ros::Subscriber goal_sub;
  goal_sub       = n.subscribe(towr_msgs::user_command, 1, towr::UserCommandCallback);
  towr::rviz_pub = n.advertise<visualization_msgs::MarkerArray>("xpp/handles", 1);

  ros::spin();

  return 1;
}
