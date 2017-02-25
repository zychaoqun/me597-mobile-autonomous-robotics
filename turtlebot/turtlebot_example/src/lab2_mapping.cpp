//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/time.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <math.h> 
#include <stdio.h>

ros::Publisher map_pub;

double ips_x;
double ips_y;
double ips_yaw;
double ips_x0;
double ips_y0;
bool is_initialized;
bool new_pose = false;
sensor_msgs::LaserScan scan;


short sgn(int x) { return x >= 0 ? 1 : -1; }

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{

    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

    if (is_initialized == false) {
        is_initialized = true;
        ips_x0 = ips_x;
        ips_y0 = ips_y;
    }

    new_pose = true;
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the Position topic (SIMULATION)
void laser_callback(const sensor_msgs::LaserScan& msg) 
{
    scan = msg;
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

// // 
// void pose_to_map(double real_x, double real_y, int map_x, int map_y, int grid_size, double resolution) {
//     map_x = floor((real_x - ips_x0) / resolution) + (map_x / 2)
//     map_y = floor((real_y - ips_y0) / resolution) + (map_y / 2)
// }

double logit(double x) {
    return log(x / (1 - x));
}

double inv_logit(double x) {
    return exp(x) / (1 + exp(x));
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/lab2_map", 1);

    int grid_size = 300;
    double resolution = 0.05; // m/cell
    double logit_p_high = logit(0.999), logit_p_low = logit(0.001), logit_p_init = logit(0.5);

    ros::Time time = ros::Time::now();
    nav_msgs::OccupancyGrid grid;

    // initialize grid
    grid.header.seq = 0;
    grid.header.stamp.sec = time.sec;
    grid.header.stamp.nsec = time.nsec;
    grid.header.frame_id = "0";

    grid.info.resolution = resolution;
    grid.info.width = grid_size;
    grid.info.height = grid_size;
    grid.info.origin.position.x = 0;
    grid.info.origin.position.y = 0;
    grid.info.origin.position.z = 0;
    grid.info.origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    grid.data = std::vector<int8_t>(grid_size * grid_size, 50); 

    //Set the loop rate
    ros::Rate loop_rate(5);

    is_initialized = false;
	
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

        ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);

        if (!is_initialized) {
            continue;
        }

        if (!new_pose) {
            continue;
        }

        // ensure that map is only updated with a updated position of the robot
        new_pose = false;

        int pose_map_x = floor((ips_x - ips_x0) / resolution) + (grid.info.width / 2);
        int pose_map_y = floor((ips_y - ips_y0) / resolution) + (grid.info.height / 2);

        // for (int i = 0; i < grid.data.size(); i++){
        //     grid.data[i] = 50;
        // }

        for (int i = 0; i < scan.ranges.size(); i++) {

            if (std::isnan(scan.ranges[i])) {
                continue;
            }

            std::vector<int> x_s;
            std::vector<int> y_s;
            double angle = ips_yaw + scan.angle_min + scan.angle_increment * i;
            double range = scan.ranges[i];
            bool exceed_range = false;

            int dx = floor(range * cos(angle) / resolution);
            int dy = floor(range * sin(angle) / resolution);

            // ROS_INFO("%d, %d, %d, %d", pose_map_x, pose_map_y, pose_map_x + dx, pose_map_y + dy);
            // ROS_INFO("%f", range);

            bresenham(pose_map_x, pose_map_y, pose_map_x + dx, pose_map_y + dy, x_s, y_s);

            for (int j = 0; j < x_s.size(); j++) {
                int grid_x = x_s[j], grid_y = y_s[j];
                int idx = grid.info.width * grid_y + grid_x;
                double logit_p = logit(grid.data[idx] / 100.0);

                // don't do anything if it is out of grid
                if (grid_x < 0 || grid_x > grid_size - 1 || grid_y < 0 || grid_y > grid_size - 1) {
                    continue;
                }

                if (j >= x_s.size() - 1 && !exceed_range) {
                    logit_p = logit_p_high + logit_p - logit_p_init;
                    // logit_p = logit(1)
                } else {
                    logit_p = logit_p_low + logit_p - logit_p_init;
                }

                grid.data[idx] = floor(inv_logit(logit_p) * 100);
            }
        }

        // update time stamp
        time = ros::Time::now();
        grid.header.stamp.sec = time.sec;
        grid.header.stamp.nsec = time.nsec;


        map_pub.publish(grid);
    }

    return 0;
}
