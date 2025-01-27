#include "planner_core.hpp"
#include <cmath>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

double PlannerCore::heuristic(const CellIndex& a, const CellIndex& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

bool PlannerCore::isValidCell(const CellIndex& cell, const nav_msgs::msg::OccupancyGrid& map) {
    if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
        cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
        return false;
    }

    const int safety_radius = 3;
    for (int dx = -safety_radius; dx <= safety_radius; dx++) {
        for (int dy = -safety_radius; dy <= safety_radius; dy++) {
            int nx = cell.x + dx;
            int ny = cell.y + dy;
            
            if (nx < 0 || nx >= static_cast<int>(map.info.width) ||
                ny < 0 || ny >= static_cast<int>(map.info.height)) {
                continue;
            }
            
            int index = ny * map.info.width + nx;
            if (map.data[index] > 15) {  
                return false;
            }
        }
    }
    return true;
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex& cell) {
    std::vector<CellIndex> neighbors;
    // Get all 8 neighbors
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            neighbors.push_back(CellIndex(cell.x + dx, cell.y + dy));
        }
    }
    return neighbors;
}

std::vector<CellIndex> PlannerCore::reconstructPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, const CellIndex& current) {
    std::vector<CellIndex> path;
    CellIndex current_cell = current;
    
    while (came_from.find(current_cell) != came_from.end()) {
        path.push_back(current_cell);
        current_cell = came_from.at(current_cell);
    }
    path.push_back(current_cell); 
    
    std::reverse(path.begin(), path.end());
    return path;
}

double PlannerCore::calculateHeuristic(const CellIndex& current, const CellIndex& goal) {
    double dx = current.x - goal.x;
    double dy = current.y - goal.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<CellIndex> PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const CellIndex& start,
    const CellIndex& goal) {
    
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    
    open_set.push(AStarNode(start, calculateHeuristic(start, goal)));
    g_score[start] = 0;
    
    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        
        if (current == goal) {
            std::vector<CellIndex> path;
            CellIndex current_cell = current;
            
            while (came_from.find(current_cell) != came_from.end()) {
                path.push_back(current_cell);
                current_cell = came_from[current_cell];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        open_set.pop();
        
        for (const auto& neighbor : getNeighbors(current)) {
            if (!isValidCell(neighbor, map)) {
                continue;
            }
            
            double movement_cost = 1.0;
            if (abs(current.x - neighbor.x) == 1 && abs(current.y - neighbor.y) == 1) {
                movement_cost = 1.414;  
            }
            
            int cell_index = neighbor.y * map.info.width + neighbor.x;
            movement_cost += map.data[cell_index] * 0.1;
            
            double tentative_g = g_score[current] + movement_cost;
            
            if (g_score.find(neighbor) == g_score.end() || 
                tentative_g < g_score[neighbor]) {
                
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                double f_score = tentative_g + calculateHeuristic(neighbor, goal);
                open_set.push(AStarNode(neighbor, f_score));
            }
        }
    }
    
    return std::vector<CellIndex>();
}

} 
