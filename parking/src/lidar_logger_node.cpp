#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <math.h>

using namespace std;
using std::placeholders::_1;

#define RAD2DEG(x) ((x) * 180. / M_PI) 

class Lidar_Logger : public rclcpp::Node
{
public:
  Lidar_Logger() : Node("Lidar_Logger")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, bind(&Lidar_Logger::topic_callback_laser, this, _1));

    subscription_odom  = this->create_subscription<nav_msgs::msg::Odometry>(
         "motorcontroller_pub", 1, bind(&Lidar_Logger::topic_callback_odom, this, _1));    

    subscription_steering  = this->create_subscription<std_msgs::msg::Float32>(
        "motorcontroller_subscriber_steering", 1, bind(&Lidar_Logger::topic_callback_steering, this, _1));        

    steering = 0;
     distance = 0;
     speed = 0;
    
    
    std::time_t result = std::time(nullptr);
    string filename("/home/ubuntu/logging/");
    filename.append(std::asctime(std::localtime(&result)));
    filename.pop_back();
    filename.append("_Output.csv");

    try
    {
      outputFile = new ofstream(filename, ios_base::out);
      filename += " open file";
      RCLCPP_INFO(this->get_logger(), filename.c_str());
      *outputFile << "Time;AngleL;DistanceL;AngleR;DistanceR;Speed;Displacement;Steering;" << endl;
    }
    catch (exception &e)
    {
      RCLCPP_INFO(this->get_logger(), e.what());
    }
  }
  //-------------------------------------------------------------------------------------------------
  ~Lidar_Logger()
  {
    if (outputFile->is_open())
      outputFile->close();
    RCLCPP_INFO(this->get_logger(), "node finished normal");
  }
  //-------------------------------------------------------------------------------------------------
private:
  void topic_callback_laser(const sensor_msgs::msg::LaserScan &scan) const
  {
    RCLCPP_DEBUG(this->get_logger(), "received scan");
    if (outputFile->is_open())
    {
      RCLCPP_DEBUG(this->get_logger(), "logged scan");
      int count = scan.scan_time / scan.time_increment;

      float tempanglel = 0, tempdistl = scan.range_max;
      float tempangler = 0, tempdistr = scan.range_max;
      for (int i = 0; i < count; i++)
      {
        float degree = RAD2DEG(scan.angle_min + scan.angle_increment * i);
        if (!isinf(scan.ranges[i]))
        {
          if ((scan.ranges[i] < tempdistl) && (scan.ranges[i] > 0.3) && (degree < 0))
          {
            tempdistl = scan.ranges[i];
            tempanglel = degree;
          }
          if ((scan.ranges[i] < tempdistr) && (scan.ranges[i] > 0.3) && (degree > 0))
          {
            tempdistr = scan.ranges[i];
            tempangler = degree;
          }
        }
      }
      auto tim = std::time(nullptr);
      string timestamp =  std::asctime(std::localtime(&tim));
      timestamp.pop_back();
      
      //"Time;AngleL;DistanceL;AngleR;DistanceR;Speed;Displacement;Steering;"
      *outputFile << timestamp << ";";
      *outputFile << to_string(tempanglel) << ";" << to_string(tempdistl) << ";";
      *outputFile << to_string(tempangler) << ";" << to_string(tempdistr) << ";";
      *outputFile << to_string(speed) << ";" << to_string(distance) << ";" << to_string(steering) << ";";
      *outputFile << std::endl;
    }
  }

  void topic_callback_odom(const nav_msgs::msg::Odometry &_odometry) 
  {
    distance = (int)_odometry.pose.pose.position.x;
    speed = (int)_odometry.twist.twist.linear.x;
  }

  void topic_callback_steering(const std_msgs::msg::Float32 &_steering) 
  {
    steering = _steering.data;
  }


  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_steering;

  ofstream *outputFile;
  float steering;
  int distance;
  int speed;

};
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar_Logger>());
  rclcpp::shutdown();
  return (0);
}
