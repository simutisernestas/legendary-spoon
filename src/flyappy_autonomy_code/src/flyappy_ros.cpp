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
    // take in double params kx,ky,kvx,kvy,front_x
    double kx, ky, kvx, kvy, front_x, vel_ref;
    nh.param<double>("kx", kx, 0.0);
    nh.param<double>("ky", ky, 0.0);
    nh.param<double>("kvx", kvx, 0.0);
    nh.param<double>("kvy", kvy, 0.0);
    nh.param<double>("front_x", front_x, 0.0);
    nh.param<double>("vel_ref", vel_ref, 0.0);

    ControlParams params{kx, ky, kvx, kvy, front_x, vel_ref};
    flyappy_.setControlParams(params);
    control_params_ = params;
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    Vec vel{msg->x, msg->y};
    flyappy_.integrateVel(vel);

    flyappy_.planPathForward();

    double ux, uy;
    flyappy_.getControlInputs(vel, ux, uy, front_range_);

    geometry_msgs::Vector3 acc_cmd{};
    acc_cmd.x = ux;
    acc_cmd.y = uy;
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
        // save 0 range to variable
        double eps = 0.05;
        if (std::abs(msg->angle_min + i * msg->angle_increment) < eps)
            front_range_ = msg->ranges[i];
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

    // reset
    flyappy_ = {};
    flyappy_.setControlParams(control_params_);
    front_range_ = 10.0;
}