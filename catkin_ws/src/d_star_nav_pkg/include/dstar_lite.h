#ifndef DSTAR_LITE_H
#define DSTAR_LITE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>

struct GridCell {
    int x, y;
    double g, rhs, cost;
    GridCell(int _x, int _y) : x(_x), y(_y), g(std::numeric_limits<double>::infinity()), 
                               rhs(std::numeric_limits<double>::infinity()), cost(1.0) {}
};

struct CellKey {
    double k1, k2;
};

struct PQItem {
    int x, y;
    double g, rhs;
    CellKey key;
};

struct ComparePQItem {
    bool operator()(const PQItem &a, const PQItem &b) const {
        if (a.key.k1 == b.key.k1)
            return a.key.k2 > b.key.k2;
        return a.key.k1 > b.key.k1;
    }
};

class DStarLite {
public:
    DStarLite();
    nav_msgs::Path plan(const geometry_msgs::Point& goal);

private:
    double heuristic(int x1, int y1, int x2, int y2);
    CellKey calculateKey(const GridCell &s, int start_x, int start_y, double km);
    void updateVertex(GridCell &u, std::vector<std::vector<GridCell>> &grid, int start_x, int start_y,
                      std::priority_queue<PQItem, std::vector<PQItem>, ComparePQItem> &open_list,
                      int width, int height);
    std::vector<std::pair<std::pair<int, int>, double>> getNeighbors(const GridCell &cell, int width, int height);
};

#endif 

