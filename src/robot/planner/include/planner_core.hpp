#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <queue>
#include <unordered_map>
#include <vector>

namespace robot {

struct CellIndex {
    int x;
    int y;
    
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    
    bool operator==(const CellIndex& other) const {
        return (x == other.x && y == other.y);
    }
    
    bool operator!=(const CellIndex& other) const {
        return (x != other.x || y != other.y);
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex& idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;
    
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode& a, const AStarNode& b) {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    
    std::vector<CellIndex> planPath(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& start, const CellIndex& goal);

  private:
    rclcpp::Logger logger_;
    
    double heuristic(const CellIndex& a, const CellIndex& b);
    bool isValidCell(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map);
    std::vector<CellIndex> getNeighbors(const CellIndex& cell);
    std::vector<CellIndex> reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, const CellIndex& current);
    
    double calculateHeuristic(const CellIndex& current, const CellIndex& goal);
    
    const double OBSTACLE_INFLATION = 0.3;  
    const int GRID_RESOLUTION = 20;         
    const double HEURISTIC_WEIGHT = 1.2;   
    
    std::vector<CellIndex> last_path_;
};

}  

#endif  
