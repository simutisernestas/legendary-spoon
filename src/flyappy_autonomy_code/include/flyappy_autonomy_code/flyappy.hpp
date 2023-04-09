#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <vector>

#define SCALING 0.01
constexpr double dt = 1.0 / 30.0;  // 30 FPS game

class OccGrid
{
  public:
    enum class Occupancy
    {
        Empty = 0,
        Obstacle = 1
    };

    OccGrid(double width_meters, double height_meters, double resolution_meters);

    Occupancy getCellAt(double x, double y) const;

    void setCellAt(Occupancy occ, double x, double y);

    void getGridCoords(double x, double y, int& grid_x, int& grid_y) const;

    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    void getMap(std::vector<Occupancy>& map) const { map = data_; };

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

struct ControlParams
{
    double kx;
    double ky;
    double kvx;
    double kvy;
    double front_x;
    double vel_ref;
};

class Flyappy
{
  public:
    Flyappy();

    void integrateVel(Vec vel);
    void getPos(Vec& pos);
    void processLaserRay(const double& distance, const double& angle);
    void planPath(Vec goal);
    void planPathForward();
    void renderViz();
    void getControlInputs(const Vec& vel, double& ux, double& uy, double fr);

    inline void getPlan(std::vector<Vec>& plan) { plan = latest_plan_; };
    inline void setControlParams(ControlParams params) { params_ = params; };

  private:
    Vec p_{56.0 * SCALING,  // taken out from game internals - starting position
           244.0 * SCALING};
    // width = 432 px * 0.01 = 4.32 [m]
    // height = 512 px * 0.01 = 5.12 [m]
    // increase width to have enough space
    OccGrid occ_grid_{4.32 * 20, 5.12, 0.2};
    std::vector<Vec> latest_plan_;
    ControlParams params_{0.97, 27.69, 1.39,
                          7.44, 2.0,   .3};  // default params from control.ipynb
};

std::vector<Vec> AStar(const OccGrid& grid, double start_x, double start_y, double goal_x,
                       double goal_y);