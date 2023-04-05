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
        : width_(static_cast<size_t>(width_meters / resolution_meters)),
          height_(static_cast<size_t>(height_meters / resolution_meters)),
          resolution_(resolution_meters),
          data_(width_ * height_, Occupancy::Empty)
    {
        // initialize grid bottom and top to obstacles
        for (size_t i = 0; i < width_; i++)
        {
            // use 21 of height for ceiling, taken from game params
            data_[index(i, static_cast<size_t>(height_ * 0.21))] = Occupancy::Obstacle;
            // use 0.79 of height for ground, taken from game params
            data_[index(i, height_ - 1)] = Occupancy::Obstacle;
        }
        // initialize grid left and right to obstacles
        for (size_t i = 0; i < height_; i++)
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
        for (size_t i = 0; i < height_; i++)
        {
            for (size_t j = 0; j < width_; j++)
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
        size_t row = static_cast<size_t>(y / resolution_);
        size_t col = static_cast<size_t>(x / resolution_);
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

        size_t row = static_cast<size_t>(y / resolution_);
        size_t col = static_cast<size_t>(x / resolution_);
        // print set at
        std::cout << "set at: " << col << " " << row << std::endl;
        // also print x,y
        std::cout << "x,y: " << x << " " << y << std::endl;
        data_[index(col, row)] = occ;
    }

    double getResolution() const { return resolution_; }
    size_t getWidth() const { return width_; }
    size_t getHeight() const { return height_; }

  private:
    size_t width_;
    size_t height_;
    double resolution_;
    std::vector<Occupancy> data_;

    size_t index(size_t x, size_t y) const { return x + width_ * y; }
};

struct Vec
{
    double x;
    double y;
};

struct Node
{
    size_t x;
    size_t y;
    double g;
    double h;
    double f;
    Node* parent;
    Node(size_t x, size_t y, double g, double h, Node* parent)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent)
    {
    }
    bool operator<(const Node& other) const { return f > other.f; }
};

class Flyappy
{
  public:
    Flyappy();

    void integrateVel(Vec vel);
    void getPos(Vec& pos);
    void processLaserRay(double distance, double angle);
    void planPath(Vec goal);
    inline void getPlan(std::vector<Node*>& plan) { plan = latest_plan_; };

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
    std::vector<Node*> latest_plan_;
};

bool areSame(double a, double b) { return fabs(a - b) < 1e-6; }

std::vector<Node*> AStar(const OccGrid& grid, double start_x, double start_y,
                         double goal_x, double goal_y)
{
    // calculate start and goal cell indices
    size_t start_row = static_cast<size_t>(start_y / grid.getResolution());
    size_t start_col = static_cast<size_t>(start_x / grid.getResolution());
    size_t goal_row = static_cast<size_t>(goal_y / grid.getResolution());
    size_t goal_col = static_cast<size_t>(goal_x / grid.getResolution());

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
    std::priority_queue<Node*> open_list;
    std::vector<Node*> closed_list;

    // add start node to open list
    open_list.push(start_node);

    // A* search
    while (!open_list.empty())
    {
        // get node with lowest f value from open list
        Node* current_node = open_list.top();
        open_list.pop();

        // check if goal is reached
        if (current_node->x == goal_node->x && current_node->y == goal_node->y)
        {
            std::vector<Node*> path;
            Node* node = current_node;
            while (node != nullptr)
            {
                path.push_back(node);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // add current node to closed list
        closed_list.push_back(current_node);

        // get neighbors of current node
        std::vector<Node*> neighbors;
        for (int row = -1; row <= 1; row++)
        {
            for (int col = -1; col <= 1; col++)
            {
                if (row == 0 && col == 0) continue;
                size_t x = current_node->x + col;
                size_t y = current_node->y + row;
                if (x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight())
                {
                    if (grid.getCellAt(x * grid.getResolution(),
                                       y * grid.getResolution()) ==
                        OccGrid::Occupancy::Empty)
                    {
                        double g = current_node->g + std::hypot(col, row);
                        auto d1 = areSame(x, goal_node->x) ? 0 : (x - goal_node->x);
                        auto d2 = areSame(y, goal_node->y) ? 0 : (y - goal_node->y);
                        double h = std::hypot(d1, d2);
                        Node* neighbor_node = new Node(x, y, g, h, current_node);
                        neighbors.push_back(neighbor_node);
                    }
                }
            }
        }

        // expand neighbors
        while (!open_list.empty())
        {
            Node* neighbor_node = open_list.top();
            open_list.pop();

            // check if neighbor is already in closed list
            bool in_closed_list = false;
            for (Node* node : closed_list)
            {
                if (node->x == neighbor_node->x && node->y == neighbor_node->y)
                {
                    in_closed_list = true;
                    break;
                }
            }
            if (in_closed_list)
            {
                delete neighbor_node;
                continue;
            }

            // calculate tentative g value for neighbor
            double tentative_g =
                    current_node->g + std::hypot(neighbor_node->x - current_node->x,
                                                 neighbor_node->y - current_node->y);

            // check if neighbor is already in open list
            bool in_open_list = false;
            std::vector<Node*> open_list_copy;
            while (!open_list.empty())
            {
                Node* node = open_list.top();
                if (node->x == neighbor_node->x && node->y == neighbor_node->y)
                {
                    in_open_list = true;
                    if (tentative_g < node->g)
                    {
                        node->g = tentative_g;
                        node->f = node->g + node->h;
                        node->parent = current_node;
                    }
                    break;
                }
                open_list_copy.push_back(node);
                open_list.pop();
            }
            for (Node* node : open_list_copy)
            {
                open_list.push(node);
            }

            // add or update neighbor in open list
            if (!in_open_list)
            {
                neighbor_node->g = tentative_g;
                neighbor_node->f = neighbor_node->g + neighbor_node->h;
                neighbor_node->parent = current_node;
                open_list.push(neighbor_node);
            }
            else
            {
                delete neighbor_node;
            }
        }
    }

    // goal not found
    return std::vector<Node*>();
}