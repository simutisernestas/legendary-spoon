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
    ROS_INFO("Position x: %f, y: %f", pos.x, pos.y);
    ROS_INFO("Velocity x: %f, y: %f", msg->x, msg->y);

    flyappy_.planPath({pos.x + 5.0, pos.y});

    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;
    acc_cmd.x = 0;
    acc_cmd.y = 0;
    pub_acc_cmd_.publish(acc_cmd);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // iterate measurements
    for (uint32_t i = 0; i < msg->ranges.size(); i++)
    {
        // check if range between min & max
        if (msg->ranges[i] <= msg->range_min || msg->ranges[i] >= msg->range_max) continue;
        flyappy_.processLaserRay(msg->ranges[i],
                                 msg->angle_min + i * msg->angle_increment);
        // print measurement
        ROS_INFO("Laser scan: %f, %f", msg->ranges[i],
                 msg->angle_min + i * msg->angle_increment);
        // print min/max laser scan message
        ROS_INFO("Laser scan min: %f, max: %f", msg->range_min, msg->range_max);
        // print min/max angles
        ROS_INFO("Laser scan angle min: %f, max: %f", msg->angle_min,
                 msg->angle_max);
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
