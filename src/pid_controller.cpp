#include "pid_controller/pid_controller.hpp"

PidController::PidController() 
{

    // initialize publishers
  cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // initialize subscribers
  odom_sub_ = n.subscribe("odom", 10, &PidController::odomMsgCallBack, this);

  // Add target points to list
  target_points.push_back(std::make_pair(2.0, 1.0));
  target_points.push_back(std::make_pair(3.0, 2.0));
  target_points.push_back(std::make_pair(4.0, 4.0));
  target_points.push_back(std::make_pair(4.0, 5.0));
  target_points.push_back(std::make_pair(6.0, 3.0));
  target_points.push_back(std::make_pair(7.0, 2.0));
  target_points.push_back(std::make_pair(8.0, -1.0));
  target_points.push_back(std::make_pair(7.0, -4.0));
  target_points.push_back(std::make_pair(4.0, -5.0));
  target_points.push_back(std::make_pair(3.0, -4.0));
  target_points.push_back(std::make_pair(2.0, -3.0));
  target_points.push_back(std::make_pair(-1.0, 1.0));
  target_points.push_back(std::make_pair(0.0, 0.0));

  kp = std::make_pair(0.2, 0.93);
  ki = std::make_pair(0.01, 0.001);
  kd = std::make_pair(0.3, 2.0);

  angel_integral = 0.0;
  distance_integral = 0.0;
  prev_error_angle = 0.0;
  prev_error_distance = 0.0;
  global_index = 0;
}

PidController::~PidController() 
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

void PidController::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg) 
{
  x_position = msg->pose.pose.position.x;
  y_position = msg->pose.pose.position.y;

  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  //ROS_INFO("Odometry: x=%f, y=%f, z=%f, w=%f", quat.x, quat.y, quat.z, quat.w);
  target_angel = tf::getYaw(quat);
  controlLoop();
}

void PidController::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

bool PidController::controlLoop() 
{

  //ROS_INFO("Target angle of the robot: %lf", target_angel);
  //ROS_INFO("Current position x: %lf", x_position);
  //ROS_INFO("Current position y: %lf", y_position);

  calculate_distance_and_angle();
  //ROS_INFO("Target points: %f %f", target_points.front().first, target_points.front().second);

  if (distance < 0.5) {
    
    global_index++;
    if(global_index == target_points.size()) {global_index = 0;}
    angel_integral = 0.0;
    distance_integral = 0.0;
    prev_error_angle = 0.0;
    prev_error_distance = 0.0;
    ROS_INFO("New target points: %f %f", target_points[global_index].first, target_points[global_index].second);
  }

  double linear_velocity = 0.0;
  double angular_velocity = 0.0;

  if(target_points.size() > 0) {
   //ROS_INFO("Distance of waypoint: %lf", distance);
    
    linear_velocity = pid_controller_distance();
    angular_velocity = pid_controller_angle();

    //ROS_INFO("Linear Velocity: %lf", linear_velocity);
    //ROS_INFO("Angular Velocity: %lf", angular_velocity);
    updatecommandVelocity(linear_velocity, angular_velocity);
  }

  return true;
}

double PidController::pid_controller_angle() {

  double error = angle - target_angel;
  //ROS_INFO("Angle of waypoint %lf", error);
  angel_integral += error;
  double derivative = error - prev_error_angle;

  if (angel_integral > 1) {angel_integral = 0;}
  prev_error_angle = error;
  return kp.second * error + kd.second * derivative + ki.second * angel_integral;
}

double PidController::pid_controller_distance() {

  //ROS_INFO("Distance's integral %lf", distance_integral);
  // Calculate errors
  double error = distance;
  distance_integral += error;
  double derivative = error - prev_error_distance;
  prev_error_distance = error;
  if (distance_integral > 3) {distance_integral = 0;}
  return  kp.first * error + kd.first * derivative + ki.first * distance_integral;
}

void PidController::calculate_distance_and_angle() {
  double target_x = target_points[global_index].first;
  double target_y = target_points[global_index].second;
  distance = std::sqrt(std::pow(x_position - target_x, 2) + std::pow(y_position - target_y, 2));
  angle = std::atan2(target_y - y_position, target_x - x_position);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_controller");
  PidController pid_controller;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
} 