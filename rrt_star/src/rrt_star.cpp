/**
 * @file rrt_star.cpp
 * @author Mustafa Izzet Mustu (mizzetmustu@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <rrt_star/rrt_star.hpp>

// Constructor
RRT_STAR::RRT_STAR(ros::NodeHandle* n): nh_(n)
{
    // Start service server and publishers
    pathService = nh_->advertiseService("/find_path", &RRT_STAR::findPath, this);
    markerPublisher = nh_->advertise<visualization_msgs::MarkerArray>("/obstacles", 1);
    pathPublisher = nh_->advertise<nav_msgs::Path>("/path", 1);

    // Get params from server
    nh_->param("obstacleNumber", obstacleNumber, 10);
    nh_->param("obstacleSize", obstacleSize, 1.0);
    nh_->param("safetyMargin", safetyMargin, 1.0);
    nh_->param("stepSize", stepSize, 1.0);
    nh_->param("goalBias", goalBias, 1.0);
    nh_->param("solutionTime", solutionTime, 0.01);
    nh_->param("goalThreshold", goalThreshold, 1.0);

    // Create random obstacle locations
    srand((unsigned) time(0));
    randomlocations = std::vector<geometry_msgs::Point>(obstacleNumber, geometry_msgs::Point());
    for(int i = 0; i < obstacleNumber; i++)
    {
        randomlocations[i].x = (rand() % 100) - 49;
        randomlocations[i].y = (rand() % 100) - 49;
    }

    // OMPL
    stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(2);
    stateSpace->as<ompl::base::RealVectorStateSpace>()->setBounds(-50,50);
    spaceInformation = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
    validityFunc = std::bind(&RRT_STAR::isStateValid, this, std::placeholders::_1);
    spaceInformation->setStateValidityChecker(std::bind(&RRT_STAR::isStateValid, this,  std::placeholders::_1));
    spaceInformation->setup();

    // Dynamic reconfigure
    f = boost::bind(&RRT_STAR::cfgCallback, this, _1, _2);
    server.setCallback(f);
    
    // Create thread
    markerThread = std::thread(&RRT_STAR::publishMarkerArray, this);
}   

// Marker publisher thread
void RRT_STAR::publishMarkerArray()
{
    ros::Rate rate(1);
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers = std::vector<visualization_msgs::Marker>(obstacleNumber);
    while(nh_->ok())
    {
        // Lock mutex, shared variables exists between callbacks
        m.lock();
        for(int i = 0; i < obstacleNumber; i++) 
        {
            markerArray.markers[i].header.frame_id = "map";
            markerArray.markers[i].header.stamp = ros::Time::now();
            markerArray.markers[i].ns = "obstacles";
            markerArray.markers[i].id = i;
            markerArray.markers[i].type = visualization_msgs::Marker::CYLINDER;
            markerArray.markers[i].action = visualization_msgs::Marker::ADD;
            markerArray.markers[i].pose.position.x = randomlocations[i].x;
            markerArray.markers[i].pose.position.y = randomlocations[i].y;
            markerArray.markers[i].pose.position.z = obstacleSize/2;
            markerArray.markers[i].pose.orientation.x = 0.0;
            markerArray.markers[i].pose.orientation.y = 0.0;
            markerArray.markers[i].pose.orientation.z = 0.0;
            markerArray.markers[i].pose.orientation.w = 1.0;
            markerArray.markers[i].scale.x = obstacleSize;
            markerArray.markers[i].scale.y = obstacleSize;
            markerArray.markers[i].scale.z = obstacleSize;
            markerArray.markers[i].color.r = 1.0f;
            markerArray.markers[i].color.g = 0.0f;
            markerArray.markers[i].color.b = 0.0f;
            markerArray.markers[i].color.a = 1.0;
            markerArray.markers[i].lifetime = ros::Duration(1.0);
        }
        while(markerPublisher.getNumSubscribers() < 1)
        {
            if(!nh_->ok())
            {
                return;
            }
            ROS_WARN_ONCE("No subscriber to the marker array");
            sleep(1);
        }
        markerPublisher.publish(markerArray);
        m.unlock();
        ros::spinOnce();
        rate.sleep();
    }
}

// Dynamic reconfigure callback
void RRT_STAR::cfgCallback(rrt_star::rrt_starConfig& config, u_int32_t level)
{
    // Lock mutex, shared variables exists between callbacks
    m.lock();
    obstacleSize = config.obstacleSize;
    safetyMargin = config.safetyMargin;
    stepSize = config.stepSize;
    goalBias = config.goalBias;
    solutionTime = config.solutionTime;
    goalThreshold = config.goalThreshold;
    m.unlock();
}

// Validity checker
bool RRT_STAR::isStateValid(const ompl::base::State *state)
{
    // Center coordinates of path
    double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    for(std::vector<geometry_msgs::Point>::iterator it = randomlocations.begin(); it != randomlocations.end(); it++)
    {
        double obstacleX = it->x;
        double obstacleY = it->y;
        double centerDistance = std::sqrt(std::pow((x - obstacleX), 2) + std::pow((y - obstacleY), 2));       
        if(centerDistance <= (obstacleSize + safetyMargin))
        {
            return false;
        }
    }
    return true;
}

// Service callback function
bool RRT_STAR::findPath(rrt_star::RRTPathFinderRequest& req, rrt_star::RRTPathFinderResponse& res)
{
    // Start timer
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now(); 

    // Convert types from ros PoseStamped to scoped state
    ompl::base::ScopedState<> startState(stateSpace);
    ompl::base::ScopedState<> goalState(stateSpace);
    startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = req.startPoint.pose.position.x;
    startState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = req.startPoint.pose.position.y;
    goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = req.goalPoint.pose.position.x;
    goalState->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = req.goalPoint.pose.position.y;
    
    // Lock mutex, shared variables exists between callbacks
    m.lock();

    // Define problem
    problemDef = std::make_shared<ompl::base::ProblemDefinition>(spaceInformation); // Create problem definition
    problemDef->setStartAndGoalStates(startState, goalState, goalThreshold);        // Set start, goal states and threshold
    ompl::base::OptimizationObjectivePtr pathLengthOptimizationObjective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(spaceInformation);
    pathLengthOptimizationObjective->setCostThreshold(ompl::base::Cost(200));       // Adjust cost threshold to Manhattan distance
    problemDef->setOptimizationObjective(pathLengthOptimizationObjective);          // Change optimizitaion objective, default cost 0

    // Create planner and set parameters
    planner = std::make_shared<ompl::geometric::RRTstar>(spaceInformation);
    planner->setProblemDefinition(problemDef);
    planner->as<ompl::geometric::RRTstar>()->setRange(stepSize);
    planner->as<ompl::geometric::RRTstar>()->setGoalBias(goalBias);
    planner->setup();

    // Try to solve
    ompl::base::PlannerStatus solved = planner->solve(solutionTime);
    m.unlock();

    // If solution exists
    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {   
        // Print path
        ompl::geometric::PathGeometric* omplPath = problemDef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        ROS_INFO_STREAM("Path found!");
        omplPath->print(std::cout);

        // Convert path to nav_msgs/Path
        nav_msgs::Path rosPath;
        rosPath.header.frame_id = "map";
        std::size_t stateCount = omplPath->getStateCount();
        for(unsigned int index = 0; index < static_cast<unsigned int>(stateCount); index++)
        {
            ompl::base::State* state = omplPath->getState(index);
            double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
            double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

            geometry_msgs::PoseStamped currentPose;
            currentPose.header.frame_id = "map";
            currentPose.header.seq = index;
            currentPose.header.stamp = ros::Time::now();
            currentPose.pose.position.x = x;
            currentPose.pose.position.y = y;
            currentPose.pose.position.z = 0.0;
            currentPose.pose.orientation.x = 0.0;
            currentPose.pose.orientation.y = 0.0;
            currentPose.pose.orientation.z = 0.0;
            currentPose.pose.orientation.w = 1.0;
            rosPath.poses.push_back(currentPose);
        }
        res.path = rosPath;
        std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
        res.serviceTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        pathPublisher.publish(rosPath);
        ros::spinOnce();
    }  
    // If solution does not exist time differs
    else
    {
        ROS_INFO_STREAM("Path could not found!");    
        std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();
        res.serviceTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();        
    }

    // Reset planner
    planner->clear();
    res.success = (solved == ompl::base::PlannerStatus::EXACT_SOLUTION);
    return (solved == ompl::base::PlannerStatus::EXACT_SOLUTION);
}   

// Deconstructor
RRT_STAR::~RRT_STAR() 
{
    markerThread.join();
}

int main(int argc, char** argv)
{
    // Init ros node
    ros::init(argc, argv, "rrt_star");
    ros::NodeHandle nh("~");

    // Create planner object
    RRT_STAR planner(&nh);
    
    // 2 callbacks, 2 threads
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin(); 

    return 0;
}