#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include "tf_conversions/tf_eigen.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
tf::TransformListener* tf_listener;
ros::Publisher tf_pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_to_map");
	ros::NodeHandle n;
	tf_listener = new tf::TransformListener();
	nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("/odom"), ros::Duration(1.0));
	nav_msgs::Odometry::ConstPtr msg_odomc = ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("/odomc"), ros::Duration(1.0));
	geometry_msgs::PoseStamped pose, transformed;
	pose.header.frame_id = msg->header.frame_id;
	pose.header.stamp = msg->header.stamp;
	pose.pose.position.x = msg->pose.pose.position.x;
	pose.pose.position.y = msg->pose.pose.position.y;
	pose.pose.position.z = msg->pose.pose.position.z;
	pose.pose.orientation.x = msg->pose.pose.orientation.x;
	pose.pose.orientation.y = msg->pose.pose.orientation.y;
	pose.pose.orientation.z = msg->pose.pose.orientation.z;
	pose.pose.orientation.w = msg->pose.pose.orientation.w;
	tf_listener->transformPose(std::string("/map"), pose, transformed);
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w);
	tf::Transform odom(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
	tf::Transform odomc(tf::Quaternion(msg_odomc->pose.pose.orientation.x, msg_odomc->pose.pose.orientation.y, msg_odomc->pose.pose.orientation.z, msg_odomc->pose.pose.orientation.w), tf::Vector3(msg_odomc->pose.pose.position.x, msg_odomc->pose.pose.position.y, msg_odomc->pose.pose.position.z));
	tf::Transform od = odomc.inverseTimes(odom);
	tf::Vector3 t = od.getOrigin();
	ROS_INFO("Translation-> x: [%f], y: [%f], z: [%f]", t[0], t[1], t[2]);
	tf::Quaternion q = od.getRotation();
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", q[0], q[1], q[2], q[3]);

	return 0;
}
