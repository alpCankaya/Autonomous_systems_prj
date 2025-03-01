#include "dstar_lite.h"

DStarLite::DStarLite() {}

nav_msgs::Path DStarLite::plan(const geometry_msgs::Point& goal) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = goal.x;
    pose.pose.position.y = goal.y;
    pose.pose.position.z = goal.z;
    path.poses.push_back(pose);

    return path;
}

double DStarLite::heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

CellKey DStarLite::calculateKey(const GridCell &s, int start_x, int start_y, double km) {
    double min_g_rhs = std::min(s.g, s.rhs);
    CellKey key;
    key.k1 = min_g_rhs + heuristic(start_x, start_y, s.x, s.y) + km;
    key.k2 = min_g_rhs;
    return key;
}

void DStarLite::updateVertex(GridCell &u, std::vector<std::vector<GridCell>> &grid, int start_x, int start_y,
                              std::priority_queue<PQItem, std::vector<PQItem>, ComparePQItem> &open_list,
                              int width, int height) {
    if (!(u.x == start_x && u.y == start_y)) {
        double min_rhs = std::numeric_limits<double>::infinity();
        std::vector<std::pair<std::pair<int, int>, double>> neighbors = getNeighbors(u, width, height);
        for (auto &nb : neighbors) {
            int nx = nb.first.first;
            int ny = nb.first.second;
            double tentative = grid[ny][nx].g + nb.second;
            if (tentative < min_rhs) {
                min_rhs = tentative;
            }
        }
        u.rhs = min_rhs;
    }
    PQItem item;
    item.x = u.x;
    item.y = u.y;
    item.g = u.g;
    item.rhs = u.rhs;
    item.key = calculateKey(u, start_x, start_y, 0.0);
    open_list.push(item);
}

std::vector<std::pair<std::pair<int, int>, double>> DStarLite::getNeighbors(const GridCell &cell, int width, int height) {
    std::vector<std::pair<std::pair<int, int>, double>> neighbors;
    int dx[8] = {-1,  0, 1, -1, 1, -1, 0, 1};
    int dy[8] = {-1, -1,-1,  0, 0,  1, 1, 1};
    for (int i = 0; i < 8; i++) {
        int nx = cell.x + dx[i];
        int ny = cell.y + dy[i];
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            double move_cost = (dx[i] == 0 || dy[i] == 0) ? 1.0 : sqrt(2);
            neighbors.push_back(std::make_pair(std::make_pair(nx, ny), move_cost));
        }
    }
    return neighbors;
}
