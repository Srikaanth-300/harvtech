#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <global_planner_f2c/f2c_planner.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <sstream>
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <pluginlib/class_list_macros.h>

using std::string;

namespace F2C_planner {

 GlobalPlanner::GlobalPlanner() {}

 /**
  * @brief: Three topics are used for visualization- global_plan (the complete plan), a_star_plan (just the path from A* algorithm) and f2c_plan (path points from CSV file).
  * The path file is given as a parameter - "/move_base/file_path"
 */
 void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ros::NodeHandle nh;
    global_plan_pub = nh.advertise<nav_msgs::Path>("global_plan", 10);
    a_star_check = nh.advertise<nav_msgs::Path>("a_star_plan", 10);
    f2c_plan_pub = nh.advertise<nav_msgs::Path>("f2c_plan", 10);
    geometry_msgs::PoseStamped robot_pose;
    costmap_ = costmap_ros;
    std::string csv_file_path; //= "/home/srikaanth/harvtech_ws/src/navigation/csv_files/simulated_boundary_f2c_1.csv";
    if (nh.getParam("/move_base/file_path", csv_file_path)) 
    {
        ROS_INFO_STREAM("CSV file path: " << csv_file_path);
    } 
    else 
    {
        ROS_ERROR("Failed to get param '/move_base/file_path");
    }
    std::ifstream file(csv_file_path);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        double x, y, z = 0.0;
        if (std::getline(ss, value, ' ')) x = std::stod(value);
        if (std::getline(ss, value, ' ')) y = std::stod(value);
        if (std::getline(ss, value, ' ')) z = std::stod(value); 
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation.w = 1.0;
        global_plan.push_back(pose);
        //ROS_INFO("x: %f ,y: %f, z: %f", x, y, z);
    }
    file.close();

    // // Read the interrupt_file (this can be used further to read data from a file where the last point was reached before the system being shut)
    // std::string interrupt_file_path = "/home/srikaanth/bison_ws/src/Fields2Cover/interrupt_coordinates.txt";
    // std::ifstream file2(interrupt_file_path);
    // if (std::getline(file2, line) && line == "true"){
    //     if(std::getline(file2, line)){
    //         std::stringstream ss1(line);
    //         std::string goals;
    //         if (std::getline(ss1, goals, ',')) interrupt_x = std::stod(goals);
    //         if (std::getline(ss1, goals, ',')) interrupt_y = std::stod(goals);
    //         ROS_INFO("The read files have values %f, %f ", interrupt_x, interrupt_y);
    //         // To be used to check where is the start point of the robot
    //         interrupted = true;
    //         //std::cout << "The interrupt variable has the value: " << interrupted << std::endl;
    //     }
    // }
    // file2.close();

    /*nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    path_msg.poses = global_plan;
    
    f2c_plan_pub.publish(path_msg);*/

    ROS_INFO("Number of waypoints in the plan: %lu", global_plan.size());
    

    ROS_INFO("F2C_planner plugin has been initiated.");

 }

 /**
  * @brief The function generates a plan for 8m ahead of the robot's current pose. It also computes the path from current pose to the starting pose from the CSV file using A*.
  * Checks if the CSV file is read and A* path is not found then, computes A* path and stores them in a_star_plan vector.
  * Then the a_star_plan vector is appended with the CSV file points and copied to complete_plan which is published on the topic f2c_plan (this is the plan for the total field).
  * Once the complete plan is generated, the a_star_calculated variable is set to true and the complete plan is given as 8m chunks to the local_controller.
  * A global variable 'last_index' is used to store the previous closest point index in complete_plan to the robot's pose.
  * From that index the closest point currently to the robot (10m from the last_index pose) is found.
  * From there on, the path (8m in length) is published and the process continues until the end is reached. Since the makePlan function is called at a frequency, the robot navigates the complete field.
  *  'cumulative_distance' variable being > 8 condition controls the path length to be published.
  */
 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    if (!global_plan.empty()){
        //geometry_msgs::PoseStamped robot_pose;
        if ((costmap_->getRobotPose(robot_pose)) && !a_star_calculated) {
            ROS_INFO("Robot pose: x = %f, y = %f", robot_pose.pose.position.x, robot_pose.pose.position.y);
            geometry_msgs::PoseStamped first_waypoint;
            first_waypoint = global_plan[0];
            // The below part is for later use as mentioned above.
            // //std::cout << "The interrupt variable inside the make_plan has the value: " << interrupted << std::endl;
            // if (interrupted){
            //     //geometry_msgs::PoseStamped first_waypoint;
            //     first_waypoint.header.stamp = ros::Time::now();
            //     first_waypoint.header.frame_id = "map";
            //     first_waypoint.pose.position.x = interrupt_x;
            //     first_waypoint.pose.position.y = interrupt_y;
            //     first_waypoint.pose.position.z = 0.0;
            //     first_waypoint.pose.orientation.w = 1.0;
            //     ROS_INFO("The first wapoints are x: %f, y: %f", interrupt_x, interrupt_y);
            // }
            // else{
            //    // geometry_msgs::PoseStamped
            //    // ROS_INFO("The first part of the plan has points x: %f, y: %f", plan[0].pose.position.x, plan[0].pose.position.y);
            //     first_waypoint = plan[0];
            //    // ROS_INFO("The first wapoints are x: %f, y: %f", first_waypoint.pose.position.x, first_waypoint.pose.position.y);
            // }
            unsigned int mx, my;
            if (costmap_->getCostmap()->worldToMap(robot_pose.pose.position.x, robot_pose.pose.position.y, mx, my)){     // convert coordinates of start from global to cells
                ROS_INFO("The grid values of the start pose's are : %u, %u", mx, my);
                std::vector<Node> a_star_path = runAStar(mx, my,first_waypoint.pose.position.x, first_waypoint.pose.position.y);
                // Convert A* path to PoseStamped and add to plan
                std::vector<geometry_msgs::PoseStamped> a_star_plan;
                for (const auto& node : a_star_path) {
                    double world_x, world_y;
                    costmap_->getCostmap()->mapToWorld(node.x, node.y, world_x, world_y);
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = world_x;
                    pose.pose.position.y = world_y;
                    pose.pose.orientation.w = 1.0;
                    a_star_plan.push_back(pose);
                    //ROS_INFO("The size of a-star plan is %lu ", a_star_plan.size());
                }
                a_star_plan.insert(a_star_plan.end(), global_plan.begin(), global_plan.end());
                total_points = a_star_plan.size();
                ROS_INFO("The size of modified plan of a-star plan is %lu ", a_star_plan.size());
                complete_plan = a_star_plan;
                a_star_calculated = true;
                nav_msgs::Path path_msg;
                path_msg.header.stamp = ros::Time::now();
                path_msg.header.frame_id = "map";
                path_msg.poses = complete_plan;
                f2c_plan_pub.publish(path_msg);
            }
            else{
                ROS_INFO("The coordinates are of starting point is of bound");
            }
        } 
        else if (a_star_calculated) {
            plan.clear();                      // To ensure no previous poses are appended
            geometry_msgs::PoseStamped wp;
            int search_start_index = last_index;
            int search_end_index = std::min((int)total_points, search_start_index + 100);    // Search for 10m from the previous pose
            double min_dist_sq = std::numeric_limits<double>::max();    // Max value to start with
            int closest_index = search_start_index;

            for (int i = search_start_index; i < search_end_index; ++i) // Loop to find the closest point from the list of points
                {
                    double dx = robot_pose.pose.position.x - complete_plan[i].pose.position.x;
                    double dy = robot_pose.pose.position.y - complete_plan[i].pose.position.y;
                    double distance_sq = dx * dx + dy * dy;
                
                    if (distance_sq < min_dist_sq)
                    {
                        min_dist_sq = distance_sq;
                        closest_index = i;
                        last_index = closest_index;
                        //ROS_INFO("last_index: %d", last_index);
                    }
                }
            double cumulative_distance = 0.0;

            for (int i =closest_index; i < (int)complete_plan.size(); ++i){

                if (i + 1 >= (int)complete_plan.size()) {  // Check to see if the index is out of bounds
                    wp.header.stamp = ros::Time::now();
                    wp.header.frame_id = "map";
                    wp.pose.position.x = complete_plan[i].pose.position.x;
                    wp.pose.position.y = complete_plan[i].pose.position.y;
                    wp.pose.orientation.w = 1.0;
                    plan.push_back(wp);
                    break;
                }

                double dx = complete_plan[i+1].pose.position.x - complete_plan[i].pose.position.x;
                double dy = complete_plan[i+1].pose.position.y - complete_plan[i].pose.position.y;
                cumulative_distance  += std::sqrt(dx * dx + dy * dy);
                
                if (cumulative_distance >= 8){    // 8m path
                    //last_index = i;
                    break;
                }
                
                wp.header.stamp = ros::Time::now();
                wp.header.frame_id = "map";
                wp.pose.position.x = complete_plan[i].pose.position.x;
                wp.pose.position.y = complete_plan[i].pose.position.y;
                wp.pose.orientation.w = 1.0;
                plan.push_back(wp);
                //ROS_INFO("The size of push point is %lu", plan.size());
                
            }
            //ROS_INFO("The size of push point is %lu", dynamic_plan.size());
            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "map";
            path_msg.poses = plan;
            //ROS_INFO("The size of plan variable is %lu", plan.size());
            global_plan_pub.publish(path_msg);
            return true;            
        }
        else{
            ROS_WARN("Could not retrieve robot pose.");
        }
    }
    else {
    ROS_WARN("Plan vector is empty!");}
    return true;    
    }
 
 /**
  * @brief Finds the shortest path between the current pose (start points) and the first pose in the CSV file (goal points).
  * Converts the pose into grid x,y values, uses a priority_queue 'openSet', unordered_map 'nodeLookup' for lookup and 'g_cost' to store the g-cost, vector 'closed_list' with initial values as false to determine if the node is visited.
  * It is a 8 direction expansion.
  * The g_cost map is used to check for already visited cells or if a cell has a lower g_cost compared to its previous parent cell
  */
 std::vector<Node> GlobalPlanner::runAStar(unsigned int start_x, unsigned int start_y, double goal_x, double goal_y) {
    
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNode> openSet;
    
    std::vector<std::vector<bool>> closed_list(costmap_->getCostmap()->getSizeInCellsX(), std::vector<bool>(costmap_->getCostmap()->getSizeInCellsY(), false)); // Stores the binary value to check if a node is already visited
    
    std::unordered_map<std::pair<unsigned int, unsigned int>, NodePtr, Hash> nodeLookup; // Stores the nodes 

    std::unordered_map<unsigned int, std::unordered_map<unsigned int, double>> g_cost; // Stores the minimum g-cost to reach a node as a nested map
    
    nav_msgs::Path nodes_path;
    nodes_path.header.stamp = ros::Time::now();
    nodes_path.header.frame_id = "map";
    
    unsigned int mx, my;

    if (costmap_->getCostmap()->worldToMap(goal_x, goal_y, mx, my)){   // Transform goal coordinates into cells
        ROS_INFO("The grid values of the goal pose's are : %u, %u", mx, my);
        ROS_INFO("The grid values of the start pose's are : %u, %u", start_x, start_y);
        ROS_INFO("Costmap size: %u x %u", costmap_->getCostmap()->getSizeInCellsX(), costmap_->getCostmap()->getSizeInCellsY());

        NodePtr start = std::make_shared<Node>();
        start->x = start_x;
        start->y = start_y;
        start->cost = 0;
        start->heuristic = heuristic(start_x, start_y, mx, my);
        start->parent = nullptr;

        openSet.push(start);

        nodeLookup[{start_x, start_y}] = start;

        g_cost[start_x][start_y] = 0;

        //ROS_INFO("Initial openSet size: %lu", openSet.size());
        ROS_INFO("Initial openSet start values: %u, %u, %f, %f, %f ", start->x, start->y, start->cost, start->heuristic, start->parent);

        std::vector<std::pair<double, double>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        //visited[start_x * 1000 + start_y] = start;

        while (!openSet.empty()) {
            NodePtr current = openSet.top();
            //ROS_INFO("Initial openSet size: %lu", openSet.size());
            openSet.pop();
            ros::Rate rate(0.5);
            //ROS_INFO("The chosen one with lowest cost is x: %u, y: %u, cost: %f", current->x, current->y, current->cost);
            //ROS_INFO("The current value being processed is x:% u, y: %u", current->x, current->y);

            // Visualization for A* 
            geometry_msgs::PoseStamped node_pose;
            double viz_x, viz_y;

            if (current->x == mx && current->y == my) {    // If the path is found
                std::vector<Node> path;
                NodePtr temp = current;
                while (temp) {
                    path.push_back(*temp);
                    temp = temp->parent;
                }
                std::reverse(path.begin(), path.end());  // Reverse the path
                ROS_INFO("Reached the target");
                bool reached_start_point = true;   // unused and can be used later
                return path;
            }

            closed_list[current->x][current->y] = true;

            for (const auto& dir : directions) {   // Expanding along 8 directions
                unsigned int new_x = current->x + dir.first;
                unsigned int new_y = current->y + dir.second;

                if (!isValid(new_x, new_y) || closed_list[new_x][new_y]){
                    continue;
                }
                double new_cost = current->cost + hypot(dir.first, dir.second);
            
                if (g_cost.find(new_x) == g_cost.end() || g_cost[new_x].find(new_y) == g_cost[new_x].end() || new_cost < g_cost[new_x][new_y]) {        // If the new cost is low or the cell hasn't been visited
                    //ROS_INFO("The present cost is %f, but the cost to this other path is %u", new_cost, g_cost[new_x][new_y]);
                    g_cost[new_x][new_y] = new_cost;

                    if (nodeLookup.find({new_x, new_y}) != nodeLookup.end()) {
                    // Node exists: update cost and parent
                    NodePtr existingNode = nodeLookup[{new_x, new_y}];
                    existingNode->cost = new_cost;
                    existingNode->parent = current;
                    openSet.push(existingNode);
                    //ROS_INFO("Modifying existing path");
                    }
                    else{
                    NodePtr neighbor = std::make_shared<Node>();
                    neighbor->x = new_x;
                    neighbor->y = new_y;
                    neighbor->cost = new_cost;
                    neighbor->heuristic = heuristic(new_x, new_y, mx, my);
                    neighbor->parent = current;
                    openSet.push(neighbor);
                   //ROS_INFO("The neighbors being pushed has the f cost: %f , and the point is x: %u, y: %u", (neighbor->heuristic + new_cost), new_x, new_y);
                    }
                    //ROS_INFO("This is the new coordinate x: %u, y: %u, cost: %f , heuristic: %f", new_x, new_y, new_cost, neighbor->heuristic);
                   // ROS_INFO("The size of openset is %lu inside the if loop", openSet.size());



                   // Visualization for A* (but it would consume lot of memory and would be slower).
                    costmap_->getCostmap()->mapToWorld(new_x, new_y, viz_x, viz_y);
                    node_pose.header.stamp = ros::Time::now();
                    node_pose.header.frame_id = "map";
                    node_pose.pose.position.x = viz_x;
                    node_pose.pose.position.y = viz_y;
                    node_pose.pose.orientation.w = 1.0;
        
                    nodes_path.poses.push_back(node_pose);
                    a_star_check.publish(nodes_path);
                }
                //rate.sleep();
            }
        }
        ROS_WARN("A* search failed to find a path.");
    }
    else{
        ROS_INFO("The goal point is out of bounds");
    }
    return {};
 }

 /**
  * @brief The function computes the equilidean distance between two points, in this case between two cells
  */
 double GlobalPlanner::heuristic(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
 }

 /**
  * @brief Checks if the currrent cell is an obstacle and returns a boolean
 */ 
 bool GlobalPlanner::isValid(unsigned int x, unsigned int y) {

    unsigned char cost = costmap_->getCostmap()->getCost(x, y);
    return cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // Avoid occupied areas
 }


}

PLUGINLIB_EXPORT_CLASS(F2C_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)
