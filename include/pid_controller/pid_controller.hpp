#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class PidController {

public:
    // Public Methodes
    PidController();
    ~PidController();
    bool controlLoop();

private:
    // ROS NodeHandle
    ros::NodeHandle n;

    // ROS Topic Publishers
    ros::Publisher cmd_vel_pub_;

    // ROS Topic Subscribers
    ros::Subscriber odom_sub_;

    // Variables
    double x_position, y_position;
    double distance, angle;
    double target_angel;
    std::vector<std::pair<double, double>> target_points;
    std::pair<double, double> kp;
    std::pair<double, double> ki;
    std::pair<double, double> kd;
    double angel_integral, distance_integral, integral;
    double prev_error_angle, prev_error_distance;
    int global_index;

    // Private Methodes
    void calculateDistanceAngle();
    void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
    void updatecommandVelocity(double linear, double angular);
    double pidControllerDistance();
    double pidControllerAngle();
};
