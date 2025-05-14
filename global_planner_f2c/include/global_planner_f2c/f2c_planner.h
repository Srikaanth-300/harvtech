#ifndef F2C_PLANNER_H
#define F2C_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <nav_msgs/Path.h>

struct Node {  // Describe the nodes used to plan the path
        unsigned int x, y;  // Coordinates
        double cost;  // g(n)
        double heuristic;  // h(n)
        std::shared_ptr<Node> parent;  // Pointer to reconstruct path

        bool operator>(const Node& other) const {
            return (cost + heuristic) > (other.cost + other.heuristic);
        }
    };
using NodePtr = std::shared_ptr<Node>;
    
struct CompareNode {
    bool operator()(const NodePtr& a, const NodePtr& b) const {
        double f_a = a->cost + a->heuristic;
        double f_b = b->cost + b->heuristic;
        
        if (f_a == f_b) {
            return a->heuristic > b->heuristic;  // Prefer lower heuristic when total cost is same for two paths
        }
        return f_a > f_b;
    }
};


struct Hash {
    size_t operator()(const std::pair<unsigned int, unsigned int>& p) const {
        return std::hash<unsigned int>()(p.first) ^ (std::hash<unsigned int>()(p.second) << 1);
    }
};

namespace F2C_planner {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    // Constructors
    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // Destructor
    virtual ~GlobalPlanner() {}
    
    // Method to initialize the planner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    // Method to create the global plan (from a CSV file)
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
    
    // A-star function
    std::vector<Node> runAStar(unsigned int start_x, unsigned int start_y, double goal_x, double goal_y);
    
    // Valid node check function
    bool isValid(unsigned int x, unsigned int y);    
    
    // Heuristic function (Euclidean distance)
    double heuristic(double x1, double y1, double x2, double y2);

    // Interrupted position
    double interrupt_x, interrupt_y = 0.0;
   

private:
    geometry_msgs::PoseStamped robot_pose;
    ros::Publisher global_plan_pub;  // Publisher to publish the global plan
    costmap_2d::Costmap2DROS* costmap_;
    std::vector<geometry_msgs::PoseStamped> global_plan;
    ros::Publisher a_star_check; //Check the plan being computed	
    std::vector<geometry_msgs::PoseStamped> complete_plan;
    int last_index = 0;
    int total_points;
    bool a_star_calculated = false;
    ros::Publisher f2c_plan_pub; //Complete f2c plan
    
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    // These three variables are unused
    bool reached_start_point = false;
    bool interrupted = false;
    bool completed_coverage = false;
};

} // namespace F2C_planner

#endif // F2C_PLANNER_H
