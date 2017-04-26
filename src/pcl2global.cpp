#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"


tf::TransformListener* tf_listener;
ros::Publisher tf_pub;

void callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
    sensor_msgs::PointCloud2 pcl_out;
    tf_listener->waitForTransform("/map", (*pcl_in).header.frame_id, (*pcl_in).header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("/map", *pcl_in, pcl_out, *tf_listener);
    tf_pub.publish(pcl_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_to_map");
  ros::NodeHandle n;
  tf_listener = new tf::TransformListener();
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/cloud_out", 1, callback);
  tf_pub = n.advertise<sensor_msgs::PointCloud2> ("/cloud_out_global", 1);

  ros::spin();

  return 0;
}
