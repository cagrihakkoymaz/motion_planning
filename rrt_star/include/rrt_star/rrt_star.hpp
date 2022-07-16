/**
 * @file rrt_star.hpp
 * @author Mustafa Izzet Mustu (mizzetmustu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <rrt_star/RRTPathFinder.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <memory>
#include <chrono>
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <rrt_star/rrt_starConfig.h>

class RRT_STAR{
private:
    // Ros objects
    ros::NodeHandle* nh_;
    ros::ServiceServer pathService;
    ros::Publisher markerPublisher;
    ros::Publisher pathPublisher;
    
    // Thread related objects
    std::thread markerThread;
    std::mutex m;

    // Parameters
    int obstacleNumber;
    double obstacleSize;
    double solutionTime;
    double safetyMargin;
    double stepSize;
    double goalBias;
    double goalThreshold;
    std::vector<geometry_msgs::Point> randomlocations;  // Obstacle locations

    // OMPL object pointers
    std::shared_ptr<ompl::base::StateSpace> stateSpace;
    std::shared_ptr<ompl::base::RealVectorBounds> bounds;
    std::shared_ptr<ompl::base::SpaceInformation> spaceInformation;
    std::function<bool(const ompl::base::State*)> validityFunc;
    std::shared_ptr<ompl::base::ProblemDefinition> problemDef;
    std::shared_ptr<ompl::base::Planner> planner;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<rrt_star::rrt_starConfig> server;
    dynamic_reconfigure::Server<rrt_star::rrt_starConfig>::CallbackType f;
public:
    RRT_STAR(ros::NodeHandle* n);   // Constructor
    void publishMarkerArray();      // Marker thread function
    bool isStateValid(const ompl::base::State *state);  // Validty checker
    bool findPath(rrt_star::RRTPathFinderRequest& req, rrt_star::RRTPathFinderResponse& res);   // Service callback
    void cfgCallback(rrt_star::rrt_starConfig& config, u_int32_t level);    //Dynamic reconfigure callback
    ~RRT_STAR();
};