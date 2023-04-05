#include "flyappy_autonomy_code/flyappy.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

Flyappy::Flyappy() = default;

void Flyappy::integrateVel(Vec vel)
{
    p_.x += vel.x * dt_;
    p_.y += vel.y * dt_;
    // TODO: remove
    std::cout << vel.x << " " << vel.y << " " << dt_ << " " << p_.x << " " << p_.y
              << std::endl;
}

void Flyappy::getPos(Vec& pos)
{
    pos.x = p_.x;
    pos.y = p_.y;
}

void Flyappy::processLaserRay(double distance, double angle)
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

void Flyappy::planPath(Vec goal) {
    auto path = AStar(occ_grid_, p_.x, p_.y, goal.x, goal.y);
    latest_plan_ = path;
}