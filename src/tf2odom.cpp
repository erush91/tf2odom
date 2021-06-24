#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2odom_publisher");

  ros::NodeHandle node;

  ros::Publisher tf_odom_pub = 
    node.advertise<nav_msgs::Odometry>("D01/tf2odom", 10);

  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  geometry_msgs::PointStamped point_msg; 

  std::string parent_frame = "world";
  std::string child_frame = "D01/imu_viz_link";

  odom_msg.header.frame_id = parent_frame; 
  odom_msg.child_frame_id = child_frame;
  ros::Rate rate(10);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(parent_frame, child_frame,  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.1).sleep();
    }
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.pose.pose.position.x = transform.getOrigin().x();
    odom_msg.pose.pose.position.y = transform.getOrigin().y();
    odom_msg.pose.pose.position.z = transform.getOrigin().z();
    tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);

    tf_odom_pub.publish(odom_msg);

    rate.sleep();
  }
  return 0;
};
