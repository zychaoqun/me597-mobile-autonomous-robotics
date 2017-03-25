//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It publishes a pose of the robot in simulation with respect 
// to the inertial frame.
//
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pose_publisher;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
    	geometry_msgs::PoseWithCovarianceStamped curpose;
	curpose.header.stamp = ros::Time::now();
	curpose.header.frame_id="/map";
	curpose.pose.pose.position = msg.pose.pose.position;
	curpose.pose.pose.orientation = msg.pose.pose.orientation;
	//republish pose for rviz
	pose_publisher.publish(curpose);
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"vis_pose_publisher");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/vis_pos", 1, true);
  

    //Set the loop rate
    ros::Rate loop_rate(40);    //40Hz update rate
	

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    }

    return 0;
}
