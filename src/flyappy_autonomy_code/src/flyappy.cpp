#include "flyappy_autonomy_code/flyappy.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

////////////////////////////////////////////
// Flyapp Implementation
////////////////////////////////////////////

Flyappy::Flyappy() = default;

void Flyappy::integrateVel(Vec vel)
{
    p_.x += vel.x * dt;
    p_.y += vel.y * dt;
}

void Flyappy::getPos(Vec& pos)
{
    pos.x = p_.x;
    pos.y = p_.y;
}

void Flyappy::processLaserRay(const double& distance, const double& angle)
{
    // convert to cartesian
    double relative_x = distance * std::cos(angle);
    double relative_y = distance * std::sin(angle);
    // convert to map
    double map_x = p_.x + relative_x;
    double map_y = p_.y + relative_y;
    // set cell to obstacle
    occ_grid_.setCellAt(OccGrid::Occupancy::Obstacle, map_x, map_y);
}

void Flyappy::planPath(Vec goal)
{
    auto path = AStar(occ_grid_, p_.x, p_.y, goal.x, goal.y);
    if (path.empty()) return;
    latest_plan_ = path;
}

void Flyappy::planPathForward()
{
    Vec goal{p_.x + params_.front_x, 2.5};
    planPath(goal);
}

void Flyappy::renderViz()
{
    std::vector<OccGrid::Occupancy> map;
    occ_grid_.getMap(map);
    int width = occ_grid_.getWidth();
    int height = occ_grid_.getHeight();

    // convert to cv::Mat
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            if (map[i * width + j] == OccGrid::Occupancy::Obstacle)
            {
                image.at<cv::Vec3b>(i, j)[0] = 255;
                image.at<cv::Vec3b>(i, j)[1] = 255;
                image.at<cv::Vec3b>(i, j)[2] = 255;
            }
        }
    }

    // draw path
    for (auto& point : latest_plan_)
    {
        double x = point.x;
        double y = point.y;
        // convert to map
        int grid_x, grid_y;
        occ_grid_.getGridCoords(x, y, grid_x, grid_y);
        // draw
        image.at<cv::Vec3b>(grid_y, grid_x)[0] = 0;
        image.at<cv::Vec3b>(grid_y, grid_x)[1] = 255;
        image.at<cv::Vec3b>(grid_y, grid_x)[2] = 0;
    }

    // resize
    cv::resize(image, image, cv::Size(), 2, 15, cv::INTER_NEAREST);
    cv::namedWindow("My Image");
    cv::imshow("My Image", image);
    cv::waitKey(10);
}

void Flyappy::getControlInputs(const Vec& vel, double& ux, double& uy)
{
    if (latest_plan_.empty()) return;

    Vec track_point;
    for (auto& point : latest_plan_)
    {
        if (point.x >= p_.x + occ_grid_.getResolution() ||
            point.y >= p_.y + occ_grid_.getResolution())
        {
            track_point = point;
            break;
        }
    }

    static Vec track_vel = {params_.vel_ref, 0.0};

    ux = -params_.kx * (p_.x - track_point.x) - params_.kvx * (vel.x - track_vel.x);
    uy = -params_.ky * (p_.y - track_point.y) - params_.kvy * (vel.y - track_vel.y);
}

////////////////////////////////////////////
// Occupancy Grid Implementation
////////////////////////////////////////////

OccGrid::OccGrid(double width_meters, double height_meters, double resolution_meters)
    : width_(static_cast<int>(width_meters / resolution_meters)),
      height_(static_cast<int>(height_meters / resolution_meters)),
      resolution_(resolution_meters),
      data_(width_ * height_, Occupancy::Empty)
{
    // use 21% of height for screen bottom, taken from game params
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

OccGrid::Occupancy OccGrid::getCellAt(double x, double y) const
{
    if (x < 0 || y < 0 || x >= width_ * resolution_ || y >= height_ * resolution_)
        throw std::out_of_range("Cell coordinates out of range");
    int row, col;
    getGridCoords(x, y, col, row);
    return data_[index(col, row)];
}

void OccGrid::setCellAt(Occupancy occ, double x, double y)
{
    if (x < 0 || y < 0 || x >= width_ * resolution_ || y >= height_ * resolution_) return;
    int row, col;
    getGridCoords(x, y, col, row);
    data_[index(col, row)] = occ;
}

void OccGrid::getGridCoords(double x, double y, int& grid_x, int& grid_y) const
{
    grid_x = static_cast<int>(x / resolution_);
    grid_y = static_cast<int>(y / resolution_);
}

////////////////////////////////////////////
// A* search implementation
////////////////////////////////////////////

struct Node
{
    int x;
    int y;
    double g;
    double h;
    double f;
    Node* parent;
    Node(int x, int y, double g, double h, Node* p)
        : x(x), y(y), g(g), h(h), f(g + h), parent(p)
    {
    }
};

struct CmpNodePtrs
{
    bool operator()(const Node* lhs, const Node* rhs) const { return lhs->f < rhs->f; }
};

std::vector<Vec> AStar(const OccGrid& grid, double start_x, double start_y, double goal_x,
                       double goal_y)
{
    // calculate start and goal cell indices
    auto grid_res = grid.getResolution();
    int start_row = static_cast<int>(start_y / grid_res);
    int start_col = static_cast<int>(start_x / grid_res);
    int goal_row = static_cast<int>(goal_y / grid_res);
    int goal_col = static_cast<int>(goal_x / grid_res);

    // check if start or goal is out of bounds
    if (start_col >= grid.getWidth() || start_row >= grid.getHeight() ||
        goal_col >= grid.getWidth() || goal_row >= grid.getHeight())
        throw std::out_of_range("Start or goal is out of bounds!");
    // don't plan it
    if (grid.getCellAt(goal_x, goal_y) == OccGrid::Occupancy::Obstacle)
        return std::vector<Vec>{};

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
    int iter_n = 0;
    while (!open_list.empty())
    {
        iter_n++;
        if (iter_n > 1000)
        {
            std::cout << "A* search exceeded 1000 iterations" << std::endl;
            break;
        }
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

            std::vector<Vec> path_meters;
            for (auto& node : path)
            {
                double x = node->x * grid_res;
                double y = node->y * grid_res + (grid_res / 2);
                path_meters.push_back({x, y});
            }
            return path_meters;
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
                    auto occ = grid.getCellAt(x * grid_res, y * grid_res);
                    if (occ == OccGrid::Occupancy::Obstacle) continue;

                    auto H = std::hypot(col, row) > 1.0 ? 10.0 : std::hypot(col, row);

                    double g = current_node->g + H;
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
    return std::vector<Vec>();
}