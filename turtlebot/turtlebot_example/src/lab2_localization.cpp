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
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <math.h> 
#include <stdio.h>
#include <random>
#include <tf/transform_broadcaster.h>

ros::Publisher map_pub;

double ips_x;
double ips_y;
double ips_yaw;
double ips_x0;
double ips_y0;
double ips_yaw0;
bool init_pose = false;
bool new_pose = false;
bool init_odom = false;

nav_msgs::Odometry odom_curr;
nav_msgs::Odometry odom_prev;

tf::TransformBroadcaster *br;
tf::Transform *tform;


//Callback function for the Position topic (SIMULATION)
// void pose_gazebo_callback(const gazebo_msgs::ModelStates& msg) 
// {

//     int i;
//     for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

//     ips_x = msg.pose[i].position.x ;
//     ips_y = msg.pose[i].position.y ;
//     ips_yaw = tf::getYaw(msg.pose[i].orientation);

//     if (init_pose == false) {
//         init_pose = true;
//         ips_x0 = ips_x;
//         ips_y0 = ips_y;
//         ips_yaw0 = ips_yaw;
//     }

//     new_pose = true;
// }

//Callback function for the Position topic (LIVE)

void pose_ips_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    if (init_pose == false) {
        init_pose = true;
        ips_x0 = ips_x;
        ips_y0 = ips_y;
        ips_yaw0 = ips_yaw;
    }

    new_pose = true;
}

//Callback function for the Position topic (SIMULATION)
void odom_callback(const nav_msgs::Odometry& msg) 
{
    odom_prev = odom_curr;
    odom_curr = msg;

    init_odom = true;
}

double pdf(double x, double mu, double sig) {
    return (1 / sqrt(2 * sig * sig * M_PI)) * exp(-((x - mu) * (x - mu)) / (2* sig * sig));
}

struct particle {
    double x;
    double y;
    double yaw;
    double weight;
    double weight_cum;
};


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"lab2_localization");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    // ros::Subscriber pose_gazebo_sub = n.subscribe("/gazebo/model_states", 1, pose_gazebo_callback);
    ros::Subscriber pose_ips_sub = n.subscribe("/indoor_pos", 1, pose_ips_callback);

    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
    ros::Publisher filter_pub = n.advertise<geometry_msgs::PoseArray>("/particle_filter", 1);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1);
    ros::Publisher ips_path_pub = n.advertise<nav_msgs::Path>("/ips_path", 1);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/robot_pose", 1);
    ros::Publisher ips_pub = n.advertise<geometry_msgs::PoseStamped>("/ips_pose", 1);

    int num_particles = 500;

    // Q Matrix
    // double Q_std_x = sqrt(0.1); //m
    // double Q_std_y = Q_std_x; 
    // double Q_std_yaw = sqrt(0.05); //rad
    double Q_std_x = 0.05; //m
    double Q_std_y = Q_std_x; 
    double Q_std_yaw = 0.09; //rad

    // R Matrix
    double R_std_x = 0.1; //m
    double R_std_y = R_std_x; 
    double R_std_yaw = 0.05; //rad
    

    std::vector<particle> particles_pred(num_particles);
    std::vector<particle> particles_est(num_particles);

    // random generators, no need for multivariate since we assume they are all independent
    std::random_device rd;
    std::default_random_engine rng(rd());
    std::normal_distribution<double> Q_dist_x(0.0, Q_std_x);
    std::normal_distribution<double> Q_dist_y(0.0, Q_std_y);
    std::normal_distribution<double> Q_dist_yaw(0.0, Q_std_yaw);

    std::normal_distribution<double> R_dist_x(0.0, R_std_x);
    std::normal_distribution<double> R_dist_y(0.0, R_std_y);
    std::normal_distribution<double> R_dist_yaw(0.0, R_std_yaw);

    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);

    ros::Time time = ros::Time::now();
    // std::string frame = "odom"; // uncomment this for lab2
    std::string frame = "map";
    geometry_msgs::PoseArray particle_vis;
    particle_vis.header.seq = 0;
    particle_vis.header.stamp.sec = time.sec;
    particle_vis.header.stamp.nsec = time.nsec;
    particle_vis.header.frame_id = frame;
    particle_vis.poses = std::vector<geometry_msgs::Pose>(num_particles);

    nav_msgs::Path path;
    path.header.seq = 0;
    path.header.stamp.sec = time.sec;
    path.header.stamp.nsec = time.nsec;
    path.header.frame_id = frame;

    nav_msgs::Path ips_path;
    ips_path.header.seq = 0;
    ips_path.header.stamp.sec = time.sec;
    ips_path.header.stamp.nsec = time.nsec;
    ips_path.header.frame_id = frame;

    //Set the loop rate
    ros::Rate loop_rate(30);

    init_pose = false;
    init_odom = false;

    // wait for the first measurement to come in
    while (ros::ok()) 
    {
        loop_rate.sleep();
        ros::spinOnce();

        ROS_INFO("pose_callback_init X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);

        if (init_pose && init_odom) {
            for (int i = 0; i < particles_est.size(); i++) {
                particles_est[i].x = ips_x0 + R_dist_x(rng);
                particles_est[i].y = ips_y0 + R_dist_y(rng);
                particles_est[i].yaw = ips_yaw0 + R_dist_yaw(rng);
            }

            new_pose = false;
            break;
        }
    }
	
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

        double vx = odom_curr.twist.twist.linear.x;
        double vy = odom_curr.twist.twist.linear.y;
        double vyaw = odom_curr.twist.twist.angular.z;

        // ROS_INFO("odom_callback vX: %g vY: %g vYaw: %g", vx, vy, vyaw);

        // ===== prediction ===== 
        for (int i = 0; i < particles_est.size(); i++) {
            double noise_x = Q_dist_x(rng);
            double noise_y = Q_dist_y(rng);
            double noise_yaw = Q_dist_yaw(rng);

            double dt = 1.0/30.0;
            double dx = vx*cos(particles_pred[i].yaw)*dt;
            double dy = vx*sin(particles_pred[i].yaw)*dt;
            double dyaw = vyaw*dt;

            particles_pred[i].x = particles_est[i].x + dx + noise_x;
            particles_pred[i].y = particles_est[i].y + dy + noise_y;
            particles_pred[i].yaw = particles_est[i].yaw + dyaw + noise_yaw;

            // handle angle roll over
            if (particles_pred[i].yaw > M_PI) {
                particles_pred[i].yaw -= 2 * M_PI;
            } 

            else if (particles_pred[i].yaw < -M_PI) {
                particles_pred[i].yaw += 2 * M_PI;
            } 
        }

        if (!new_pose) {
            for (int i = 0; i < particles_est.size(); i++) {
                particles_est[i] = particles_pred[i];
            }
        } else {

            ROS_ERROR("pose_callback X: %g Y: %g Yaw: %g", ips_x, ips_y, ips_yaw);
            new_pose = false;

            time = ros::Time::now();
            geometry_msgs::PoseStamped ips_pose_stamped;
            ips_pose_stamped.header.seq = 0;
            ips_pose_stamped.header.stamp.sec = time.sec;
            ips_pose_stamped.header.stamp.nsec = time.nsec;
            ips_pose_stamped.header.frame_id = frame;
            ips_pose_stamped.pose.position.x = ips_x;
            ips_pose_stamped.pose.position.y = ips_y;
            ips_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(ips_yaw);
            
            ips_path.poses.push_back(ips_pose_stamped);

            ips_pub.publish(ips_pose_stamped);
            ips_path_pub.publish(ips_path);


            // ===== update ===== 
            double weight_sum = 0;
            for (int i = 0; i < particles_pred.size(); i++) {
                double p_x = pdf(ips_x, particles_pred[i].x, R_std_x);
                double p_y = pdf(ips_y, particles_pred[i].y, R_std_y);
                double p_yaw = pdf(ips_yaw, particles_pred[i].yaw, R_std_yaw);

                // we can just multiply them because we assume they are independent
                particles_pred[i].weight = p_x * p_y * p_yaw;
                weight_sum += particles_pred[i].weight;
            }

            // normalize and calculate cumulative weights
            double weight_cum = 0;
            for (int i = 0; i < particles_pred.size(); i++) {
                weight_cum += particles_pred[i].weight / weight_sum;
                particles_pred[i].weight_cum = weight_cum;
            }

            // importance sampling
            for (int i = 0; i < particles_pred.size(); i++) {
                double seed = uniform_dist(rng); // generate random number from zero to one

                // find the first particle with the cumulative weight > seed
                for (int j = 0; j < particles_pred.size(); j++) {
                    if (particles_pred[j].weight_cum > seed) {
                        particles_est[i] = particles_pred[j];
                        particles_est[i].weight = 1;
                        break;
                    }
                }
            }
        }

        // find mean and variance of particles
        double mean_x = 0, mean_y = 0, mean_yaw = 0, mean_cos_yaw = 0, mean_sin_yaw = 0;
        double var_x = 0, var_y = 0;
        double ss_x = 0, ss_y = 0;
        double n = particles_est.size();

        for (int i = 0; i < particles_est.size(); i++) {
            mean_x += particles_est[i].x;
            mean_y += particles_est[i].y;
            mean_cos_yaw += cos(particles_est[i].yaw);
            mean_sin_yaw += sin(particles_est[i].yaw);
            
            ss_x += particles_est[i].x * particles_est[i].x;
            ss_y += particles_est[i].y * particles_est[i].y;
        }

        mean_x = mean_x / n;
        mean_y = mean_y / n;
        mean_yaw = atan2(mean_sin_yaw / n, mean_cos_yaw / n);

        var_x = ss_x / n - mean_x * mean_x;
        var_y = ss_y / n - mean_y * mean_y;

        ROS_WARN("pose_est X: %f Y: %f Yaw: %f", mean_x, mean_y, mean_yaw);
        ROS_WARN("pose_var X: %f Y: %f", var_x, var_y);

        // output to visualization
        for (int i = 0; i < particles_est.size(); i++) {
            particle_vis.poses[i].position.x = particles_est[i].x;
            particle_vis.poses[i].position.y = particles_est[i].y;
            particle_vis.poses[i].orientation = tf::createQuaternionMsgFromYaw(particles_est[i].yaw);
        }

        time = ros::Time::now();
        particle_vis.header.stamp.sec = time.sec;
        particle_vis.header.stamp.nsec = time.nsec;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.seq = 0;
        pose_stamped.header.stamp.sec = time.sec;
        pose_stamped.header.stamp.nsec = time.nsec;
        pose_stamped.header.frame_id = frame;
        pose_stamped.pose.position.x = mean_x;
        pose_stamped.pose.position.y = mean_y;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(mean_yaw);


        path.poses.push_back(pose_stamped);
        path_pub.publish(path);

        filter_pub.publish(particle_vis);
        pose_pub.publish(pose_stamped);

        // send transform
        // br = new tf::TransformBroadcaster;
        // tform = new tf::Transform;
        // tform->setOrigin( tf::Vector3(mean_x, mean_y, 0) );
        // tf::Quaternion q;
        // q.setEulerZYX(mean_yaw, 0, 0);
        // tform->setRotation( q );
        // *tform = tform->inverse();
        // br->sendTransform(tf::StampedTransform(*tform, ros::Time::now(), "base_footprint", "map"));

    }

    return 0;
}
