#include "pid_controller/pid_controller.hpp"

// Constructor for the PidController class
PidController::PidController() 
{

  // Create a publisher to publish command velocity messages to the "cmd_vel" topic
  cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Create a subscriber to listen to odometry messages from the "odom" topic
  odom_sub_ = n.subscribe("odom", 10, &PidController::odomMsgCallBack, this);

  // Create a subscriber to listen to odometry messages from the "odom" topic
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

  // Initialize PID controller gains
  kp = std::make_pair(0.2, 0.93); // proportional gains for distance and angle
  ki = std::make_pair(0.01, 0.001); // integral gains for distance and angle
  kd = std::make_pair(0.3, 2.0); // derivative gains for distance and angle

  // Initialize integral and previous error values
  angel_integral = 0.0;
  distance_integral = 0.0;
  prev_error_angle = 0.0;
  prev_error_distance = 0.0;

  // Initialize global index to keep track of current target point
  global_index = 0;
}

// Destructor for the PidController class
PidController::~PidController() 
{
  // Stop the robot by sending a command velocity of 0
  updatecommandVelocity(0.0, 0.0);

  // Shutdown the ROS node
  ros::shutdown();
}

// Callback methode for odometry messages
void PidController::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg) 
{
  // Get the current position and orientation from the odometry message
  x_position = msg->pose.pose.position.x;
  y_position = msg->pose.pose.position.y;

  // Convert the quaternion orientation to a yaw angle
  geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
  target_angel = tf::getYaw(quat);

  // Call the control loop function
  controlLoop();
}

// Methode to update the command velocity
void PidController::updatecommandVelocity(double linear, double angular)
{
  // Create a Twist message to publish to the "cmd_vel" topic
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

bool PidController::controlLoop() 
{
  // Calculate the distance and angle to the current target point
  calculateDistanceAngle();

  // Check if the robot has reached the current target point
  if (distance < 0.5) 
  {
    
    global_index++;
    if(global_index == target_points.size()) {global_index = 0;}

    // Reset the integral and previous error values
    angel_integral = 0.0;
    distance_integral = 0.0;
    prev_error_angle = 0.0;
    prev_error_distance = 0.0;
    ROS_INFO("New target points: %f %f", target_points[global_index].first, target_points[global_index].second);
  }

  // Calculate the linear and angular velocities using PID control
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;

  // Calculate the linear/angular velocity using the distance PID controller
  linear_velocity = pidControllerDistance();
  angular_velocity = pidControllerAngle();

  // Update the command velocity
  updatecommandVelocity(linear_velocity, angular_velocity);
  
  return true;
}

// PID controller for the angle
double PidController::pidControllerAngle() 
{
  // Calculate the error between the current angle and the target angle
  double error = angle - target_angel;

  angel_integral += error;
  double derivative = error - prev_error_angle;

  if (angel_integral > 1) {angel_integral = 0;}
  prev_error_angle = error;
  return kp.second * error + kd.second * derivative + ki.second * angel_integral;
}

// PID controller for the distance
double PidController::pidControllerDistance() 
{
  double error = distance;

  distance_integral += error;
  double derivative = error - prev_error_distance;
  prev_error_distance = error;
  if (distance_integral > 3) {distance_integral = 0;}
  return  kp.first * error + kd.first * derivative + ki.first * distance_integral;
}

// Function to calculate the distance and angle to the current target point
void PidController::calculateDistanceAngle() 
{
  double target_x = target_points[global_index].first;
  double target_y = target_points[global_index].second;
  distance = std::sqrt(std::pow(x_position - target_x, 2) + std::pow(y_position - target_y, 2));
  angle = std::atan2(target_y - y_position, target_x - x_position);
}

int main(int argc, char **argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "pid_controller");
  PidController pid_controller;

  // Set the loop rate to 10 Hz
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Spin once to process any incoming messages
    ros::spinOnce();

    // Sleep for the remaining time to maintain the loop rate
    loop_rate.sleep();
  }


  return 0;
} 