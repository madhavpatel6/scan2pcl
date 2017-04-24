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

#include <string>
tf::TransformListener* tf_listener;
ros::Publisher tf_pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
    sensor_msgs::PointCloud2 pcl_out;
    //tf_listener->waitForTransform("/odom", (*pcl_in).header.frame_id, (*pcl_in).header.stamp, ros::Duration(5.0));
    std::string topic = "/odomc";
    nav_msgs::Odometry::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::Odometry>(std::string("/odomc"), ros::Duration(1.0));
    //ROS_INFO("Seq: [%d]", msg->header.seq);
    //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    tf::Quaternion _q = tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Vector3 _tr = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::Transform _t = tf::Transform(_q, _tr);
    tf::StampedTransform t = tf::StampedTransform(_t, msg->header.stamp, std::string("/odom"), std::string("base_link"));
    Eigen::Matrix4f T;
    pcl_ros::transformAsMatrix(t, T);

    pcl_ros::transformPointCloud(T, *pcl_in, pcl_out);
    tf_pub.publish(pcl_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_to_map");
  ros::NodeHandle n;
  tf_listener = new tf::TransformListener();
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_out", 1, callback);
  tf_pub = n.advertise<sensor_msgs::PointCloud2> ("/icp/scan2", 1);

  ros::spin();

  return 0;
}
