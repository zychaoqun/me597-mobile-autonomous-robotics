//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h> 
#include <stdio.h>
#include <algorithm> 


#define TAGID 0

ros::Publisher nodes_pub;
ros::Publisher edges_pub;
nav_msgs::OccupancyGrid occupancy_grid;


//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double X = msg.pose.pose.position.x; // Robot X psotition
	double Y = msg.pose.pose.position.y; // Robot Y psotition
 	double Yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with

    occupancy_grid = msg;
}

struct Point {
    double x;
    double y;

    geometry_msgs::Point viz(const nav_msgs::OccupancyGrid& grid) {
        geometry_msgs::Point pt;
        pt.x = x * grid.info.resolution + grid.info.origin.position.x;
        pt.y = y * grid.info.resolution + grid.info.origin.position.y;
        pt.z = 0;
        return pt;
    } 
};

struct Node;

struct Edge {
    double dist;
    Node *node_from;
    Node *node_to;
};

struct Node  {
    Point pt;
    Node *prev;
    std::vector<Edge> edges;
};

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//    vectors of integers and shold be defined where this function is called from.
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

bool check_collision(const nav_msgs::OccupancyGrid &grid, Point start, Point end) {
    std::vector<int> x;
    std::vector<int> y;

    int sample_every_other = 1;

    bresenham(start.x, start.y, end.x, end.y, x, y);

    for (int i = 0; i < x.size(); i+=sample_every_other) {
        int idx = grid.info.width * y[i] + x[i];

        // if there is a collision
        if (grid.data[idx] > 50) {
            return false;
        }
    }

    return true;
}

double node_dist(const Node &n1, const Node &n2) {
    // std::cout << sqrt((n1.pt.x - n2.pt.x) * (n1.pt.x - n2.pt.x) + (n1.pt.y - n2.pt.y) * (n1.pt.y - n2.pt.y)) << std::endl;
    return sqrt((n1.pt.x - n2.pt.x) * (n1.pt.x - n2.pt.x) + (n1.pt.y - n2.pt.y) * (n1.pt.y - n2.pt.y));
}

void prm_find_path(const nav_msgs::OccupancyGrid &grid, Point start_pt, Point end_pt, std::vector<Point> &way_points) {

    ros::Time tic = ros::Time::now();
    int n_samples = 100;
    int n_cloest_nodes = 10;
    double prm_gaussian_sample_std_dev = 0.75; // meters

    // random generators
    std::random_device rd;
    std::default_random_engine rng(812734019273);
    std::normal_distribution<double> prm_gaussian_sample(0.0, prm_gaussian_sample_std_dev);
    std::uniform_int_distribution<int> width_dist(0, grid.info.width);
    std::uniform_int_distribution<int> height_dist(0, grid.info.height);

    std::vector<Node> milestones;

    visualization_msgs::Marker valid_nodes_viz, invalid_nodes_viz, start_node_viz, end_node_viz;
    visualization_msgs::Marker edge_base_viz;
    visualization_msgs::MarkerArray edges_viz;

    // visualization stuff
    valid_nodes_viz.header.frame_id = "/map";
    valid_nodes_viz.id = 0;
    valid_nodes_viz.type = visualization_msgs::Marker::SPHERE_LIST;
    valid_nodes_viz.action = visualization_msgs::Marker::ADD;
    valid_nodes_viz.ns = "valid_nodes";
    valid_nodes_viz.scale.x = 0.1;
    valid_nodes_viz.scale.y = 0.1;
    valid_nodes_viz.scale.z = 0.1;
    valid_nodes_viz.color.r = 0.0;
    valid_nodes_viz.color.g = 1.0;
    valid_nodes_viz.color.b = 0.0;
    valid_nodes_viz.color.a = 1.0;

    invalid_nodes_viz = valid_nodes_viz;
    invalid_nodes_viz.id = 1;
    invalid_nodes_viz.ns = "invalid_nodes";
    invalid_nodes_viz.color.r = 1.0;
    invalid_nodes_viz.color.g = 0.0;
    invalid_nodes_viz.color.b = 0.0;

    start_node_viz = valid_nodes_viz;
    start_node_viz.id = 2;
    start_node_viz.ns = "start_node";
    start_node_viz.scale.x = 0.25;
    start_node_viz.scale.y = 0.25;
    start_node_viz.scale.z = 0.25;
    start_node_viz.color.r = 0.0;
    start_node_viz.color.g = 0.0;
    start_node_viz.color.b = 1.0;

    end_node_viz = start_node_viz;
    end_node_viz.id = 3;
    end_node_viz.ns = "end_node";
    end_node_viz.scale.x = 0.15;
    end_node_viz.scale.y = 0.15;
    end_node_viz.scale.z = 0.15;

    edge_base_viz = valid_nodes_viz;
    edge_base_viz.type = visualization_msgs::Marker::LINE_STRIP;
    edge_base_viz.action = visualization_msgs::Marker::ADD;
    edge_base_viz.color.r = 0.0;
    edge_base_viz.color.g = 0.5;
    edge_base_viz.color.b = 0.0;
    edge_base_viz.color.a = 0.25;
    edge_base_viz.scale.x = 0.02;

    // add the start and end nodes
    Node start_node, end_node;
    start_node.pt.x = (start_pt.x - grid.info.origin.position.x) / grid.info.resolution;
    start_node.pt.y = (start_pt.y - grid.info.origin.position.y) / grid.info.resolution;
    end_node.pt.x = (end_pt.x - grid.info.origin.position.x) / grid.info.resolution;
    end_node.pt.y = (end_pt.y - grid.info.origin.position.y) / grid.info.resolution;
    milestones.push_back(start_node);
    milestones.push_back(end_node);
    start_node_viz.points.push_back(start_node.pt.viz(grid));
    end_node_viz.points.push_back(end_node.pt.viz(grid));

    // generate milestones
    int n_gen = 0;
    while (n_gen < n_samples) {
        int x1 = width_dist(rng);
        int y1 = height_dist(rng);
        int idx1 = grid.info.width * y1 + x1;

        int x2 = (int) round(prm_gaussian_sample(rng) / grid.info.resolution) + x1;
        int y2 = (int) round(prm_gaussian_sample(rng) / grid.info.resolution) + y1;
        if (x2 < 0 || x2 > grid.info.width - 1 || y2 < 0 || y2 > grid.info.height - 1) {
            continue;
        }
        int idx2 = grid.info.width * y2 + x2;

        Point pt;
        if ((grid.data[idx1] < 50) && (grid.data[idx2] < 50)) {
            continue;
        }
        else if ((grid.data[idx1] >= 50) && (grid.data[idx2] >= 50)) {
            continue;
        } else if (grid.data[idx1] < 50) {
            pt.x = x1;
            pt.y = y1;
        } else if (grid.data[idx2] < 50) {
            pt.x = x2;
            pt.y = y2;
        }

        // Point pt;
        geometry_msgs::Point pt_viz;
        // pt.x = x1;
        // pt.y = y1;
        pt_viz = pt.viz(grid);

        // if cell is empty
        // if (grid.data[idx1] < 50) {
        Node n;
        n.pt = pt;
        milestones.push_back(n);
        n_gen++;

        valid_nodes_viz.points.push_back(pt_viz); // visualization
        // } else {
        //     invalid_nodes_viz.points.push_back(pt_viz); // visualization
        // }
    }

    std::vector<Node> milestones_sorted = milestones;
    for (int i = 0; i < milestones.size(); i++) {

        auto sort_milestones_by_closeness = [&](const Node &n1, const Node &n2) {
            double d1 = node_dist(milestones[i], n1);
            double d2 = node_dist(milestones[i], n2);
            return d1 < d2;
        };
        
        std::sort(milestones.begin(), milestones.begin() + i, sort_milestones_by_closeness);

        for (int j = 0; j < std::min(i, n_cloest_nodes); j++) {
            // check if edge has a collision
            if (check_collision(grid, milestones[i].pt, milestones[j].pt)) {
                Edge e1, e2;
                e1.node_from = &milestones[i];
                e1.node_to = &milestones[j];
                e1.dist = node_dist(milestones[i], milestones[j]);

                e2.node_from = e1.node_to;
                e2.node_to = e1.node_from;
                e2.dist = e1.dist;

                milestones[i].edges.push_back(e1);
                milestones[j].edges.push_back(e2);

                // visualization, only need one (bidirectional) edge
                visualization_msgs::Marker edge_viz = edge_base_viz;
                geometry_msgs::Point pt_viz1 = milestones[i].pt.viz(grid), pt_viz2 = milestones[j].pt.viz(grid);
                edge_viz.id = edges_viz.markers.size();
                edge_viz.points.push_back(pt_viz1);
                edge_viz.points.push_back(pt_viz2);
                edges_viz.markers.push_back(edge_viz);

                // ROS_INFO("Edges (%f, %f), (%f, %f), dist=%f, i=%d j=%d", pt_viz1.x, pt_viz1.y, pt_viz2.x, pt_viz2.y, node_dist(milestones[i], milestones[j]), i, j);
            }
        }
    }

    // implement A*
    // auto sort_milestones_by_closeness = [&](const Node &n1, const Node &n2) {
    //     double d1 = node_dist(milestones[i], n1);
    //     double d2 = node_dist(milestones[i], n2);
    //     return d1 < d2;
    // };

    // // find shortest path
    // std::priority_queue<Node*> open_list;


    // hile

    // publish visualization
    edges_pub.publish(edges_viz);
    nodes_pub.publish(invalid_nodes_viz);
    nodes_pub.publish(valid_nodes_viz);
    nodes_pub.publish(start_node_viz);
    nodes_pub.publish(end_node_viz);

    ros::Time toc = ros::Time::now();
    ROS_INFO("PRM Completed in %f seconds", (toc - tic).toSec());
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    nodes_pub = n.advertise<visualization_msgs::Marker>("prm_nodes", 1, true);
    edges_pub = n.advertise<visualization_msgs::MarkerArray>("prm_edges", 1, true);
    // marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(1);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

        Point start, end;
        start.x = 0;
        start.y = 0;
        end.x = 4;
        end.y = 0;
        std::vector<Point> way_points;
        prm_find_path(occupancy_grid, start, end, way_points);


    	// velocity_publisher.publish(vel); // Publish the command velocity


        ROS_INFO("Hello");
    }

    return 0;
}
