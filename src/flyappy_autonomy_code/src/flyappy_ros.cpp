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
    Vec vel{msg->x, msg->y};
    flyappy_.integrateVel(vel);
    Vec pos{};
    flyappy_.getPos(pos);
    flyappy_.planPath({pos.x + 5.0, 2.5});
    // flyappy_.renderViz();

    std::vector<Vec> path;
    flyappy_.getPlan(path);
    Vec track_point = path[2];
    track_point.y += .15; // half of resolution
    Vec track_vel = {0.0, 0.0};

    auto ux = -0.9761376149468296 * (pos.x - track_point.x) - 1.4309296608236848 * (vel.x - track_vel.x);
    auto uy = -9.28017830620232 * (pos.y - track_point.y) - 4.318156819963757 * (vel.y - track_vel.y);
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
