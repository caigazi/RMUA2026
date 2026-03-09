#ifndef _BASIC_DEV_CPP_
#define _BASIC_DEV_CPP_

#include "basic_dev.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "basic_dev"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    BasicDev go(&n);
    return 0;
}

BasicDev::BasicDev(ros::NodeHandle *nh)
{  
    //创建图像传输控制句柄
    it = std::make_unique<image_transport::ImageTransport>(*nh); 
    nh->param("takeoff_wait_s", takeoff_wait_s, 3.0);
    nh->param("forward_speed_mps", forward_speed_mps, 5.0);
    nh->param("forward_duration_s", forward_duration_s, 10.0);
    nh->param("publish_rate_hz", publish_rate_hz, 20.0);
    nh->param("log_telemetry", log_telemetry, false);
    nh->param("velocity_topic", velocity_topic, std::string("/airsim_node/drone_1/vel_body_cmd"));
    nh->param("velocity_acceleration", velocity_acceleration, 8);

    front_left_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));
    front_right_img = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0));

    takeoff.request.waitOnLastTask = 1;
    land.request.waitOnLastTask = 1;

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    velcmd.vx = 0.0;
    velcmd.vy = 0.0;
    velcmd.vz = 0.0;
    velcmd.yawRate = 0.0;
    if(velocity_acceleration < 0)
    {
        velocity_acceleration = 0;
    }
    else if(velocity_acceleration > 8)
    {
        velocity_acceleration = 8;
    }
    velcmd.va = static_cast<uint8_t>(velocity_acceleration);
    velcmd.stop = 0;

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&BasicDev::pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&BasicDev::gps_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&BasicDev::imu_cb, this, std::placeholders::_1));//imu数据
    lidar_suber = nh->subscribe<sensor_msgs::PointCloud2>("airsim_node/drone_1/lidar", 1, std::bind(&BasicDev::lidar_cb, this, std::placeholders::_1));//imu数据
    // front_left_view_suber = it->subscribe("airsim_node/drone_1/front_left/Scene", 1, std::bind(&BasicDev::front_left_view_cb, this,  std::placeholders::_1));
    // front_right_view_suber = it->subscribe("airsim_node/drone_1/front_right/Scene", 1, std::bind(&BasicDev::front_right_view_cb, this,  std::placeholders::_1));
    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    takeoff_client = nh->serviceClient<airsim_ros::Takeoff>("/airsim_node/drone_1/takeoff");
    land_client = nh->serviceClient<airsim_ros::Land>("/airsim_node/drone_1/land");
    reset_client = nh->serviceClient<airsim_ros::Reset>("/airsim_node/reset");
    //通过publisher实现对无人机的速度控制和姿态控制和角速度控制
    vel_publisher = nh->advertise<airsim_ros::VelCmd>(velocity_topic, 10);
    ROS_INFO("Velocity command topic: %s", velocity_topic.c_str());
    ROS_INFO("Velocity acceleration parameter: %d", velocity_acceleration);

    control_thread = std::thread(&BasicDev::run_control, this);

    ros::spin();
}

BasicDev::~BasicDev()
{
    stop_requested = true;
    if(control_thread.joinable())
    {
        control_thread.join();
    }
}

void BasicDev::run_control()
{
    ros::Duration(1.0).sleep();

    if(!takeoff_client.waitForExistence(ros::Duration(5.0)))
    {
        ROS_ERROR("Takeoff service is unavailable.");
        return;
    }

    if(!vel_publisher)
    {
        ROS_ERROR("Velocity publisher is unavailable.");
        return;
    }

    const ros::Time wait_subscriber_start = ros::Time::now();
    while(ros::ok() && !stop_requested && vel_publisher.getNumSubscribers() == 0)
    {
        if((ros::Time::now() - wait_subscriber_start).toSec() > 5.0)
        {
            ROS_WARN("No subscriber on velocity topic after 5 seconds, continuing anyway.");
            break;
        }

        ROS_INFO_THROTTLE(1.0, "Waiting for AirSim to subscribe to %s...", velocity_topic.c_str());
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Velocity topic subscriber count: %u", vel_publisher.getNumSubscribers());

    ROS_INFO("Calling takeoff service.");
    if(!takeoff_client.call(takeoff) || !takeoff.response.success)
    {
        ROS_ERROR("Takeoff failed.");
        return;
    }

    ROS_INFO("Takeoff succeeded. Waiting %.1f s before forward flight.", takeoff_wait_s);
    ros::Duration(takeoff_wait_s).sleep();

    airsim_ros::VelCmd forward_cmd = velcmd;
    forward_cmd.vx = forward_speed_mps;

    ros::Rate rate(publish_rate_hz);
    const ros::Time forward_start = ros::Time::now();
    ROS_INFO("Flying forward at %.2f m/s for %.1f s.", forward_speed_mps, forward_duration_s);
    while(ros::ok() && !stop_requested && (ros::Time::now() - forward_start).toSec() < forward_duration_s)
    {
        forward_cmd.header.stamp = ros::Time::now();
        vel_publisher.publish(forward_cmd);
        ROS_INFO_THROTTLE(1.0, "Publishing forward command: vx=%.2f m/s, vy=%.2f m/s, vz=%.2f m/s",
            forward_cmd.vx,
            forward_cmd.vy,
            forward_cmd.vz);
        rate.sleep();
    }

    publish_hover_command();
    ROS_INFO("Forward flight complete. Hover command sent.");
}

void BasicDev::publish_hover_command()
{
    airsim_ros::VelCmd hover_cmd = velcmd;
    hover_cmd.vx = 0.0;
    hover_cmd.vy = 0.0;
    hover_cmd.vz = 0.0;
    hover_cmd.yawRate = 0.0;
    hover_cmd.stop = 0;

    ros::Rate rate(publish_rate_hz);
    for(int i = 0; ros::ok() && !stop_requested && i < 20; ++i)
    {
        hover_cmd.header.stamp = ros::Time::now();
        vel_publisher.publish(hover_cmd);
        rate.sleep();
    }
}

void BasicDev::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!log_telemetry)
    {
        return;
    }

    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO_THROTTLE(1.0, "Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!log_telemetry)
    {
        return;
    }

    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO_THROTTLE(1.0, "Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void BasicDev::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(log_telemetry)
    {
        ROS_INFO_THROTTLE(1.0, "Get imu data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
}

void BasicDev::front_left_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_left_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_left_ptr->image.empty())
    {
        ROS_INFO("Get front left image.: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
}

void BasicDev::front_right_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_front_right_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    if(!cv_front_right_ptr->image.empty())
    {
        ROS_INFO("Get front right image.%f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
    }
}

void BasicDev::lidar_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pts);
    if(log_telemetry)
    {
        ROS_INFO_THROTTLE(1.0, "Get lidar data. time: %f, size: %ld", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9, pts->size());
    }
}

#endif