//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos. 2012 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#define STATE_INIT          0
#define STATE_GO_STRAIGHT   1
#define STATE_WAIT          2
#define STATE_TURN          3

double yaw, x, y;

//Callback function for the Position topic 
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    //This function is called when a new pose message is received

    double X = msg.pose.pose.position.x; // Robot X psotition
    double Y = msg.pose.pose.position.y; // Robot Y psotition
    double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    yaw = Yaw + M_PI;
    x = X;
    y = Y;

    ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", x, y, yaw);
}



int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    // change log level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(5);    //20Hz update rate

    int state = STATE_INIT;
    double target_yaw, init_x, init_y, dist, err_curr = 0, err_prev = 0;
    double w = 0, v = 0;
    const double SPEED = 0.05;
    const double SQ_DIST = 0.4;
    const double P_CONST = 0.1;
    
    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    
        if (state == STATE_INIT) {
            target_yaw = yaw;
            init_x = x;
            init_y = y;
            state = STATE_GO_STRAIGHT;
        }

        if (state == STATE_GO_STRAIGHT) {
            dist = sqrt((x - init_x) * (x - init_x) + (y - init_y) * (y - init_y));

            if (dist > SQ_DIST) {
                target_yaw += M_PI / 4;

                if (target_yaw > M_PI) {
                    target_yaw -= M_PI;
                }

                v = 0;

                state = STATE_TURN;
            } else {
                v = SPEED;
            }

            ROS_DEBUG("Main - Target: dist=%f, yaw=%f", dist, target_yaw);
        }


        if (state == STATE_TURN) {
            err_prev = err_curr;
            err_curr = target_yaw - yaw;

            if (fabs(err_prev - err_curr) < 0.01 && fabs(err_curr) < 0.05) {
                state = STATE_GO_STRAIGHT;
                init_x = x;
                init_y = y;
                w = 0;
            } else {
                w = err_curr * P_CONST;
            }

            ROS_DEBUG("Main - Target: dist=%f, yaw=%f, delta=%f", dist, target_yaw, fabs(err_prev - err_curr));
        }

        //Main loop code goes here:
        vel.linear.x = v; // set linear speed
        vel.angular.z = w; // set angular speed

        velocity_publisher.publish(vel); // Publish the command velocity
        ROS_DEBUG("Main - Velocity commands: v=%f, w=%f", vel.linear.x, vel.angular.z);
    }

    return 0;
}
