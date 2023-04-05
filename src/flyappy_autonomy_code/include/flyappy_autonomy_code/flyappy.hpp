#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <vector>

#define SCALING 0.01

class OccGrid
{
  public:
    enum class Occupancy
    {
        Empty = 0,
        Obstacle = 1
    };

    OccGrid(double width_meters, double height_meters, double resolution_meters)
        : width_(static_cast<int>(width_meters / resolution_meters)),
          height_(static_cast<int>(height_meters / resolution_meters)),
          resolution_(resolution_meters),
          data_(width_ * height_, Occupancy::Empty)
    {
        // use 21 of height for screen bottom, taken from game params
        int floor_bound = static_cast<int>(height_ * 0.21);
        // initialize grid bottom and top to obstacles
        for (int i = 0; i < width_; i++)
        {
            data_[index(i, floor_bound)] = Occupancy::Obstacle;
            data_[index(i, height_ - 1)] = Occupancy::Obstacle;
        }
        // initialize grid left and right to obstacles
        for (int i = 0; i < height_; i++)
        {
            data_[index(0, i)] = Occupancy::Obstacle;
            data_[index(width_ - 1, i)] = Occupancy::Obstacle;
        }
    }

    ~OccGrid()
    {
        // dump occupancy grid to file to visualize
        std::ofstream file;
        file.open("/workspaces/flyappy_autonomy_test_public/occ_grid.txt");
        for (int i = 0; i < height_; i++)
        {
            for (int j = 0; j < width_; j++)
            {
                // write 0 if empty, 1 if obstacle and seperate cells by spaces
                file << static_cast<int>(data_[index(j, i)]) << " ";
            }
            file << std::endl;
        }
    }

    Occupancy getCellAt(double x, double y) const
    {
        if (x < 0 || y < 0 || x >= width_ * resolution_ || y >= height_ * resolution_)
        {
            std::cout << "x: " << x << " y: " << y << " width: " << width_
                      << " height: " << height_ << " resolution: " << resolution_
                      << std::endl;
            throw std::out_of_range("Cell coordinates out of range");
        }
        int row = static_cast<int>(y / resolution_);
        int col = static_cast<int>(x / resolution_);
        return data_[index(col, row)];
    }

    void setCellAt(Occupancy occ, double x, double y)
    {
        if (x < 0 || y < 0 || x >= width_ * resolution_ || y >= height_ * resolution_)
        {
            // print debug info
            std::cout << "x: " << x << " y: " << y << " width: " << width_
                      << " height: " << height_ << " resolution: " << resolution_
                      << std::endl;
            throw std::out_of_range("Cell coordinates out of range");
        }

        int row = static_cast<int>(y / resolution_);
        int col = static_cast<int>(x / resolution_);
        // print set at
        std::cout << "set at: " << col << " " << row << std::endl;
        // also print x,y
        std::cout << "x,y: " << x << " " << y << std::endl;
        data_[index(col, row)] = occ;
    }

    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

  private:
    int width_;
    int height_;
    double resolution_;
    std::vector<Occupancy> data_;

    int index(int x, int y) const { return x + width_ * y; }
};

struct Vec
{
    double x;
    double y;
};

struct Node
{
    int x;
    int y;
    double g;
    double h;
    double f;
    Node* parent;
    Node(int x, int y, double g, double h, Node* parent)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent)
    {
    }
};

struct CmpNodePtrs
{
    bool operator()(const Node* lhs, const Node* rhs) const { return lhs->f < rhs->f; }
};

class Flyappy
{
  public:
    Flyappy();

    void integrateVel(Vec vel);
    void getPos(Vec& pos);
    void processLaserRay(double distance, double angle);
    void planPath(Vec goal);
    inline void getPlan(std::vector<Vec>& plan) { plan = latest_plan_; };

  private:
    double dt_{1.0 / 30.0};  // 30 FPS game
    Vec p_{56.0 * SCALING,
           244.0 * SCALING};  // taken out from game internals - starting position
    // SCREENWIDTH  = 432 px
    // SCREENHEIGHT = 512 px
    // SCALING = 0.01
    // BASEY = SCREENHEIGHT * 0.79
    // width = 432 * 0.01 = 4.32 [m]
    // height = 512 * 0.01 = 5.12 [m]
    // increase width by 100 to have enough space
    OccGrid occ_grid_{4.32 * 100, 5.12, 0.1};
    std::vector<Vec> latest_plan_;
};

std::vector<Vec> AStar(const OccGrid& grid, double start_x, double start_y, double goal_x,
                       double goal_y)
{
    // calculate start and goal cell indices
    int start_row = static_cast<int>(start_y / grid.getResolution());
    int start_col = static_cast<int>(start_x / grid.getResolution());
    int goal_row = static_cast<int>(goal_y / grid.getResolution());
    int goal_col = static_cast<int>(goal_x / grid.getResolution());

    // check if start or goal is out of bounds
    if (start_col >= grid.getWidth() || start_row >= grid.getHeight() ||
        goal_col >= grid.getWidth() || goal_row >= grid.getHeight())
    {
        throw std::out_of_range("Start or goal is out of bounds!");
    }

    // create start and goal nodes
    Node* start_node = new Node(start_col, start_row, 0, 0, nullptr);
    Node* goal_node = new Node(goal_col, goal_row, 0, 0, nullptr);

    // initialize open and closed lists
    std::vector<Node*> open_list;
    open_list.reserve(100);
    std::vector<Node*> closed_list;
    closed_list.reserve(100);

    // add start node to open list
    open_list.push_back(start_node);

    std::vector<Node*> path{};

    // A* search
    while (!open_list.empty())
    {
        // get node with lowest f value from open list
        Node* current_node = open_list[0];
        open_list.erase(open_list.begin());

        // add current node to closed list
        closed_list.push_back(current_node);

        // check if goal is reached
        if (current_node->x == goal_node->x && current_node->y == goal_node->y)
        {
            Node* node = current_node;
            while (node != nullptr)
            {
                path.push_back(node);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        // iterate over neighbors
        for (int row = -1; row <= 1; row++)
        {
            for (int col = -1; col <= 1; col++)
            {
                // skip current node
                if (row == 0 && col == 0) continue;

                int x = current_node->x + col;
                int y = current_node->y + row;

                // check if in bounds
                if (x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight())
                {
                    // check if neighbor is obstacle
                    auto occ = grid.getCellAt(x * grid.getResolution(),
                                              y * grid.getResolution());
                    if (occ == OccGrid::Occupancy::Obstacle) continue;

                    double g = current_node->g + std::hypot(col, row);
                    double h = std::hypot(goal_node->x - x, goal_node->y - y);
                    Node* neighbor_node = new Node(x, y, g, h, current_node);

                    // find if neightbor x,y matches
                    auto it = std::find_if(closed_list.begin(), closed_list.end(),
                                           [x, y](const Node* n)
                                           { return n->x == x && n->y == y; });
                    if (it != closed_list.end())
                    {
                        // skip this neighbor
                        continue;
                    }

                    // check if neighbor is already in open list with lower f value
                    it = std::find_if(open_list.begin(), open_list.end(),
                                      [x, y](const Node* n)
                                      { return n->x == x && n->y == y; });
                    if (it != open_list.end())
                    {
                        // update f value of neighbor node if it has a lower f value in
                        // the open list
                        if ((*it)->f > neighbor_node->f)
                        {
                            (*it)->f = neighbor_node->f;
                            (*it)->parent = neighbor_node->parent;
                        }
                        // skip this neighbor
                        continue;
                    }

                    // add neighbor to open list
                    open_list.push_back(neighbor_node);
                }
            }
        }  // end for

        // sort open list so the lowest cost is in the front
        std::sort(open_list.begin(), open_list.end(), CmpNodePtrs());
    }

    // convert path to meters
    std::vector<Vec> path_meters;
    for (auto& node : path)
    {
        double x = node->x * grid.getResolution();
        double y = node->y * grid.getResolution();
        path_meters.push_back({x, y});
    }

    // goal not found
    return path_meters;
}