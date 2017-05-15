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
ros::Publisher tf_pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_to_map");

	ros::NodeHandle n;
	ros::Duration(2).sleep();
	tf::TransformListener _tf;
    tf::Transformer tflooker;
    ros::Duration(2).sleep();
    tf::StampedTransform transform;
	while(true){
	nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("/odom"), ros::Duration(1.0));
	nav_msgs::Odometry::ConstPtr msg_odomc = ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("/odomc"), ros::Duration(1.0));
	geometry_msgs::PoseStamped transformed, pose;

	pose.pose = msg->pose.pose;
	pose.header.stamp = msg->header.stamp;
	pose.header.frame_id = msg->header.frame_id;
	//tflooker.lookupTransform(std::string("/odom"), std::string("/map"), ros::Time(0), transform);
	try{
	ROS_INFO("Original Pose-> x: [%f], y: [%f], z: [%f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	ROS_INFO("Original Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
	_tf.transformPose(std::string("/map"), pose, transformed);
	tf::StampedTransform trf;
	_tf.lookupTransform("odom", "map", ros::Time(0), trf);

	tf::Vector3 tr = trf.getOrigin();
	ROS_INFO("transformListener: T x: [%f], y: [%f], z: [%f]", tr[0], tr[1], tr[2]);
	tf::Quaternion q = trf.getRotation();
	ROS_INFO("transformListener: R x: [%f], y: [%f], z: [%f], w: [%f]", q[0], q[1], q[2], q[3]);

	ROS_INFO("Transformed Pose-> x: [%f], y: [%f], z: [%f]", transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z);
	ROS_INFO("Transformed Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w);
	}
	catch(...) {
	}
    }
	return 0;
}
