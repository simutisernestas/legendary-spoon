#include "flyappy_autonomy_code/flyappy.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

Flyappy::Flyappy() = default;

void Flyappy::integrateVel(Vec vel)
{
    p_.x += vel.x * dt_;
    p_.y += vel.y * dt_;
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

void Flyappy::planPath(Vec goal)
{
    auto path = AStar(occ_grid_, p_.x, p_.y, goal.x, goal.y);
    latest_plan_ = path;
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
    cv::waitKey((1.0 / 30.0) * 1000);
}