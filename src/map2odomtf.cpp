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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
using namespace sensor_msgs;
using namespace message_filters;
static tf::TransformBroadcaster* br;

void callback(const nav_msgs::Odometry::ConstPtr& odom, const nav_msgs::Odometry::ConstPtr& odomc)
{
  // Solve all of perception here...
    tf::Transform odomtf, odomctf;

    tf::poseMsgToTF(odom->pose.pose, odomtf);
    tf::poseMsgToTF(odomc->pose.pose, odomctf);

    tf::Quaternion odom_q = odomtf.getRotation();
    tf::Quaternion odomc_q = odomctf.getRotation();
    tf::Vector3 odom_t = odomtf.getOrigin();
    tf::Vector3 odomc_t = odomctf.getOrigin();

    tf::Quaternion tf_q = odom_q;
    tf_q *= odomc_q.inverse();
    tf::Matrix3x3 tf_rot = tf::Matrix3x3(tf_q);

    tf::Vector3 tf_tr = odom_t;
    tf::Vector3 sub = tf::Vector3(tf_rot.getRow(0).dot(odomc_t), tf_rot.getRow(1).dot(odomc_t), tf_rot.getRow(2).dot(odomc_t));
    tf_tr -= sub;

    tf::Transform od(tf_q, tf_tr);
    tf::Vector3 t = od.getOrigin();
    tf::Quaternion q = od.getRotation();
    ROS_INFO("Translation-> x: [%f], y: [%f], z: [%f]", t[0], t[1], t[2]);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]\n", q[0], q[1], q[2], q[3]);
    br->sendTransform(tf::StampedTransform(od, odom->header.stamp, "odom", "map"));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Map2OdomTransformPublisher");

    ros::NodeHandle nh;
    br = new tf::TransformBroadcaster();
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odomc_sub(nh, "odomc", 1);
    TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(odom_sub, odomc_sub, 1);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
/*
int main(int argc, char** argv)
{
	ros::init(argc, argv, "point_cloud_to_map");
	ros::NodeHandle n;
	tf_listener = new tf::TransformListener();
	while(true){
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
	try{
	tf_listener->transformPose(std::string("/map"), pose, transformed);
	ROS_INFO("Original Pose-> x: [%f], y: [%f], z: [%f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
	ROS_INFO("Original Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
	ROS_INFO("Transformed Pose-> x: [%f], y: [%f], z: [%f]", transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z);
	ROS_INFO("Transformed Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", transformed.pose.orientation.x, transformed.pose.orientation.y, transformed.pose.orientation.z, transformed.pose.orientation.w);
	}
	catch(...) {
	}
	/*tf::Transform odom(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w), tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
	tf::Transform odomc(tf::Quaternion(msg_odomc->pose.pose.orientation.x, msg_odomc->pose.pose.orientation.y, msg_odomc->pose.pose.orientation.z, msg_odomc->pose.pose.orientation.w), tf::Vector3(msg_odomc->pose.pose.position.x, msg_odomc->pose.pose.position.y, msg_odomc->pose.pose.position.z));
	//tf::Transform odom, odomc;
		tf::Transform od = odomc.inverseTimes(odom);
	tf::Vector3 t = od.getOrigin();
	ROS_INFO("Translation1-> x: [%f], y: [%f], z: [%f]", t[0], t[1], t[2]);
	tf::Quaternion q = od.getRotation();
	ROS_INFO("Orientation1-> x: [%f], y: [%f], z: [%f], w: [%f]", q[0], q[1], q[2], q[3]);*//*
	tf::Transform odom, odomc;
	tf::poseMsgToTF(msg->pose.pose, odom);
	tf::poseMsgToTF(msg_odomc->pose.pose, odomc);
	tf::Transform od = odomc.inverseTimes(odom);
	tf::Vector3 t = od.getOrigin();
	ROS_INFO("Translation2-> x: [%f], y: [%f], z: [%f]", t[0], t[1], t[2]);
	tf::Quaternion q = od.getRotation();
	ROS_INFO("Orientation2-> x: [%f], y: [%f], z: [%f], w: [%f]", q[0], q[1], q[2], q[3]);
    }
	return 0;
}
*/