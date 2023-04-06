#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    static int counter = 0;
    counter++;

    Vec vel{msg->x, msg->y};
    flyappy_.integrateVel(vel);
    Vec pos{};
    flyappy_.getPos(pos);
    flyappy_.planPath({pos.x + 3.0, 2.5});
    // if (counter % 10 == 0) flyappy_.renderViz();
    // flyappy_.renderViz();

    std::vector<Vec> path;
    flyappy_.getPlan(path);
    if (path.empty()) return;
    Vec track_point = path[1];
    track_point.y += .125;  // half of resolution
    Vec track_vel = {.3, 0.0};

    auto ux = -0.9767059149738836 * (pos.x - track_point.x) -
              1.3976451015719364 * (vel.x - track_vel.x);
    auto uy = -27.699915176080157 * (pos.y - track_point.y) -
              7.443106230073582 * (vel.y - track_vel.y);
    geometry_msgs::Vector3 acc_cmd{};
    acc_cmd.x = ux;
    acc_cmd.y = uy;
    ROS_INFO("Publish acceleration command x: %f, y: %f", acc_cmd.x, acc_cmd.y);
    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // iterate measurements
    for (uint32_t i = 0; i < msg->ranges.size(); i++)
    {
        // check if range between min & max
        if (msg->ranges[i] <= msg->range_min || msg->ranges[i] >= msg->range_max)
            continue;
        flyappy_.processLaserRay(msg->ranges[i],
                                 msg->angle_min + i * msg->angle_increment);
    }
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }
}
