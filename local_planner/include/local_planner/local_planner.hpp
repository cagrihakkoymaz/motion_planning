/**
 * @file local_planner.hpp
 * @author Mustafa Izzet Mustu (mizzetmustu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <rrt_star/RRTPathFinder.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <mutex>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>  

#define PI 3.14159265

class LocalPlanner
{
private:
    // ROS objects
    ros::NodeHandle* nh_;
    ros::ServiceClient client_;
    ros::Subscriber pathSubscriber;
    std::string map_frame;

    // Car parameters
    double d;
    double l;
    double threshold;
    double yaw;
    double start_x;
    double start_y;
    double current_x;
    double current_y;
    double current_yaw;

    // PID parameters
    double Kp;
    double Ki;
    double Kd;
    double Kpa;
    double Kia;
    double Kda;

    // Thread for car spawner
    std::thread carThread;
    std::mutex m;

    // TF objects
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
public:
    LocalPlanner(ros::NodeHandle* n);   // Constructor
    void carTF();   // Car spawn thread function
    double distanceCalculator(double srcX, double srcY, double dstX, double dstY); // Distance calculator
    void pathCallback(const nav_msgs::PathConstPtr& pathmsg);           // Callback function
    ~LocalPlanner(){};  // Destructor
};
