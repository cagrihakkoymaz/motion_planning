/**
 * @file local_planner.cpp
 * @author Mustafa Izzet Mustu (mizzetmustu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <local_planner/local_planner.hpp>

// Constructor
LocalPlanner::LocalPlanner(ros::NodeHandle *n) : nh_(n)
{
    // Subscribe path and create server client(unnecessary)
    client_ = nh_->serviceClient<rrt_star::RRTPathFinder>("/path");
    pathSubscriber = nh_->subscribe("/path", 1, &LocalPlanner::pathCallback, this);

    // Get parameters from server
    nh_->param<std::string>("map_frame", map_frame, "map");
    nh_->param("d", d, 3.6);
    nh_->param("l", l, 2.5);
    nh_->param("threshold", threshold, 1.0);
    nh_->param("yaw", yaw, 0.0);
    nh_->param("start_x", start_x, 0.0);
    nh_->param("start_y", start_y, 0.0);
    nh_->param("Kp", Kp, 1.0);
    nh_->param("Ki", Ki, 0.0);
    nh_->param("Kd", Kd, 0.0);
    nh_->param("Kpa", Kpa, 1.0);
    nh_->param("Kia", Kia, 0.0);
    nh_->param("Kda", Kda, 0.0);

    // Set current position and orientation
    current_x = start_x;
    current_y = start_y;
    current_yaw = yaw;

    // Start spawn car on the map
    carThread = std::thread(&LocalPlanner::carTF, this);
}

// Spawn car in map
void LocalPlanner::carTF()
{
    ros::Rate tfRate(10);
    tf::Transform transform;
    tf::Quaternion q;
    while (nh_->ok())
    {
        m.lock();
        transform.setOrigin(tf::Vector3(current_x, current_y, 1.166));
        q.setRPY(0, 0, (current_yaw)*PI/180);
        transform.setRotation(q);
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_frame, "body"));
        m.unlock();
        tfRate.sleep();
    }
}

// Distance calculator
double LocalPlanner::distanceCalculator(double srcX, double srcY, double dstX, double dstY)
{
    return std::sqrt(std::pow((dstX - srcX), 2) + std::pow((dstY - srcY), 2));
}

// Service callback
void LocalPlanner::pathCallback(const nav_msgs::PathConstPtr &pathmsg)
{
    // Find middle of the front wheels, convert to bicycle model 
    double fm_x;    
    double fm_y;
    double fm_yaw;
    tf::StampedTransform transform;
    listener.lookupTransform("body", "front_middle_wheel", ros::Time(0), transform);
    fm_x = transform.getOrigin().getX();
    fm_y = transform.getOrigin().getY();
    tf::Matrix3x3 mm = transform.getBasis();
    double rr, pp, yy;
    mm.getRPY(rr, pp, yy);
    fm_yaw = yy*180/PI;
    ROS_WARN_STREAM(fm_x << " " << fm_y << fm_yaw);


    ros::Rate rate(10);
    nav_msgs::Path path = *pathmsg;
    double lastTime = ros::Time::now().toSec();
    double startTime = ros::Time::now().toSec();
    double dt = 0;
    double integral = 0;
    double integralRot = 0;
    double derivative = 0;
    double derivativeRot = 0;

    // Iterate through path points
    for (std::vector<geometry_msgs::PoseStamped>::iterator it = path.poses.begin(); it != path.poses.end(); it++)
    {
        // Debug purpose
        ROS_ERROR("path point x %f y %f", it->pose.position.x, it->pose.position.y);

        // Calculate current distance and orientation to the target point in the path from current location
        double currDistance = distanceCalculator(fm_x, fm_y, it->pose.position.x, it->pose.position.y);
        double prevDistance = currDistance;
        double desiredAngle = std::atan2(it->pose.position.y - fm_y, it->pose.position.x - fm_x) * 180 / PI;
        double prevAngle = (fm_yaw + current_yaw);

        // Try to reach current path point
        while (currDistance > threshold)
        {
            startTime = ros::Time::now().toSec();
            dt = startTime - lastTime;
            lastTime = startTime;

            m.lock();
            
            // Calculate linear velocity 
            derivative = (currDistance - prevDistance) / dt;
            integral = integral + (currDistance - prevDistance) * dt;
            prevDistance = currDistance;
            double linearVelocity = std::min((currDistance * Kp + Ki * integral + Kd * derivative), 4.0);

            // Calculate angular velocity
            derivativeRot = ((fm_yaw + current_yaw) - prevAngle) / dt;
            integralRot = integralRot + ((fm_yaw + current_yaw) - prevAngle) * dt;
            prevAngle = (fm_yaw + current_yaw);
            double angularVelocity = (desiredAngle - (fm_yaw + current_yaw)) * Kpa + Kia * integralRot + Kda * derivativeRot;
        
            // Rotate front wheel, 45 degree is maximum for steering gear ratio of 20 and 2.5 tour of steering wheel
            fm_yaw += angularVelocity*dt;
            if(fm_yaw >= 45)
            {
                fm_yaw = 45;
            }
            else if (fm_yaw <=-45)
            {
                fm_yaw = -45;
            }

            // Changes in front middle wheel
            double mx = (linearVelocity * dt) * cos((current_yaw + fm_yaw)*PI / 180);
            double my = (linearVelocity * dt) * sin((current_yaw + fm_yaw)*PI / 180);
            fm_x += mx;
            fm_y += my;

            // Changes in center of the car body
            double myaw =  (linearVelocity * dt) * sin(fm_yaw*PI/180)/l*180/PI;
            current_yaw +=  myaw;         
            current_x = fm_x - l*cos(current_yaw*PI/180);
            current_y = fm_y - l*sin(current_yaw*PI/180);

            // Update wheel rotation
            fm_yaw = fm_yaw - myaw;
            if(fm_yaw >= 45)
            {
                fm_yaw = 45;
            }
            else if (fm_yaw <=-45)
            {
                fm_yaw = -45;
            }

            // Debug porpuses
            ROS_WARN("fm_yaw: %f", fm_yaw);  
            ROS_WARN("currentYaw: %f", current_yaw);  
           
            m.unlock();

            // Update distance and desired angle
            currDistance = distanceCalculator(fm_x, fm_y, it->pose.position.x, it->pose.position.y);
            desiredAngle = std::atan2(it->pose.position.y - fm_y, it->pose.position.x - fm_x) * 180 / PI;

            // Debug porpuses
            ROS_ERROR("DESIRED ANGLE %f", desiredAngle);
            
            rate.sleep();
        }
    }
}

// Main function
int main(int argc, char **argv)
{
    // Initialize ros node
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh("~");

    // Create planner object
    LocalPlanner planner(&nh);
    ros::spin();
    return 0;
}