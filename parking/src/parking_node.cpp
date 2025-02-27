#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>

/* OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "parking/parking.hpp"

using namespace cv;
using namespace std;

using std::placeholders::_1;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

Parking::Parking()
    : Node("parking"),
      autonomous_mode(0), cross_parking(false), parallel_parking(false), parking_maneuver(false), navigation_speed(1.0), navigation_steering(1.0)
{
  RCLCPP_INFO(this->get_logger(), "Create Parking ");
  readParametersFromYaml();

  vector<CrossParkingIn::TransitionConditions> crossstates;
  crossstates.resize(CrossParkingIn::CrossParkingStates::ENUM_LAST);
  CrossParkingIn::MaxValuesCrossParking crossmaxvalues;

  readParametersCrossInFromYaml(crossstates, crossmaxvalues);

  if (cross_in.Setup(crossmaxvalues, crossstates))
    RCLCPP_INFO(this->get_logger(), "Cross_in setup done ");
  else
    RCLCPP_ERROR(this->get_logger(), "Cross_in setup Error ");

  vector<ParallelParkingIn::TransitionConditions> parallelstates;
  parallelstates.resize(ParallelParkingIn::ParallelParkingStates::ENUM_LAST);
  ParallelParkingIn::MaxValuesParallelParking parallelmaxvalues;

  readParametersParallelInFromYaml(parallelstates, parallelmaxvalues);

  if (parallel_in.Setup(parallelmaxvalues, parallelstates))
    RCLCPP_INFO(this->get_logger(), "Parallel_in setup done ");
  else
    RCLCPP_ERROR(this->get_logger(), "Parallel_in setup Error ");    

  // Read the parameter file in .txt form -> redundant with the yaml functionality
  /*   std::ifstream inputFile("./src/parking/include/parking/parking_params.txt");
    if (!inputFile)
    {
      std::cerr << "Error opening the file." << std::endl;
    }
    double value;
    while (inputFile >> value)
      parking_params.push_back(value);
    inputFile.close(); */
  // Subscriptions

  // Motorcontroller
  sub_motorcontroller_turns = this->create_subscription<std_msgs::msg::Int32>("/motorcontroller_pub_turns", 1, std::bind(&Parking::callbackMotorTurns, this, _1));

  // GUI
  sub_gui_mode = this->create_subscription<std_msgs::msg::Int32>("/gui_control/mode", 1, std::bind(&Parking::callbackGUIMode, this, _1));
  sub_gui_parallel_parking = this->create_subscription<std_msgs::msg::Bool>("/gui_control/cross_parking", 1, std::bind(&Parking::callbackCrossParking, this, _1));
  sub_gui_cross_parking = this->create_subscription<std_msgs::msg::Bool>("/gui_control/parallel_parking", 1, std::bind(&Parking::callbackParallelParking, this, _1));
  sub_gui_parking_maneuver = this->create_subscription<std_msgs::msg::Bool>("/gui_control/parking_maneuver", 1, std::bind(&Parking::callbackGUIParkingManeuver, this, _1));
  sub_gui_parking_in = this->create_subscription<std_msgs::msg::Bool>("/gui_control/parking_in", 1, std::bind(&Parking::callbackParkingIn, this, _1));
  sub_gui_parking_out = this->create_subscription<std_msgs::msg::Bool>("/gui_control/parking_out", 1, std::bind(&Parking::callbackParkingOut, this, _1));

  // Pixelbased Navigation and Hough Transformation
  sub_cam_speed_ = this->create_subscription<std_msgs::msg::Float32>("/camera_navigation_node/throttle", 1, std::bind(&Parking::callbackCAMSpeed, this, _1));
  sub_cam_steering_ = this->create_subscription<std_msgs::msg::Float32>("/camera_navigation_node/steering", 1, std::bind(&Parking::callbackCAMSteering, this, _1));
  // Neural Networks
  sub_nn_speed_ = this->create_subscription<std_msgs::msg::Float32>("/NN_Drive/throttle", 1, std::bind(&Parking::callbackCAMSpeed, this, _1));
  sub_nn_steering_ = this->create_subscription<std_msgs::msg::Float32>("/NN_Drive/steering", 1, std::bind(&Parking::callbackCAMSteering, this, _1));

  // Lidar and IMU
  
  sub_lidar_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&Parking::callbackLidarScan, this, _1));
  sub_imu = this->create_subscription<std_msgs::msg::Float32MultiArray>("/imu_euler", 1, std::bind(&Parking::callbackImu, this, _1));

  // Ultrasonic sensors
  subscription_us_0 = this->create_subscription<std_msgs::msg::Float32>("ultrasonic_sensor_0", 1, std::bind(&Parking::topic_callback_us_0, this, _1));
  subscription_us_1 = this->create_subscription<std_msgs::msg::Float32>("ultrasonic_sensor_1", 1, std::bind(&Parking::topic_callback_us_1, this, _1));
  subscription_us_2 = this->create_subscription<std_msgs::msg::Float32>("ultrasonic_sensor_2", 1, std::bind(&Parking::topic_callback_us_2, this, _1));
  subscription_us_3 = this->create_subscription<std_msgs::msg::Float32>("ultrasonic_sensor_3", 1, std::bind(&Parking::topic_callback_us_3, this, _1));
  subscription_us_4 = this->create_subscription<std_msgs::msg::Float32>("ultrasonic_sensor_4", 1, std::bind(&Parking::topic_callback_us_4, this, _1));
  subscription_us_5 = this->create_subscription<std_msgs::msg::Float32>("ultrasonic_sensor_5", 1, std::bind(&Parking::topic_callback_us_5, this, _1));

  // Publishers
  motor_speed_pub = this->create_publisher<std_msgs::msg::Float32>("/parking/throttle", 1);
  steering_pub = this->create_publisher<std_msgs::msg::Float32>("/parking/steering", 1);
  publisher_image = this->create_publisher<sensor_msgs::msg::Image>("/sensor_image", 1);

  // Timer for periodic tasks
  timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Parking::timerCallback, this));
  RCLCPP_INFO(this->get_logger(), "Create Parking done ");
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackGUIMode(const std_msgs::msg::Int32::SharedPtr msg)
{
  autonomous_mode = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackCrossParking(const std_msgs::msg::Bool::SharedPtr msg)
{
  cross_parking = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackParallelParking(const std_msgs::msg::Bool::SharedPtr msg)
{
  parallel_parking = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackGUIParkingManeuver(const std_msgs::msg::Bool::SharedPtr msg)
{
  parking_maneuver = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackCAMSpeed(const std_msgs::msg::Float32::SharedPtr msg)
{
  navigation_speed = msg->data;
}
//----------------------------------------------------------------------------------------------------
void Parking::callbackCAMSteering(const std_msgs::msg::Float32::SharedPtr msg)
{
  navigation_steering = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  lidar = *scan;

  float range_max = scan->range_max;
  // float lidar_average_distance = 0.0;

  // Scaling factor to fit the LaserScan data within the image (2 meters)
  // comment out for full range
  range_max = 3.0;

  if (lidar_init == false)
  {
    lidar_info = new LidarInfo;
    lidar_info->lidar_scale = img_size / (2.0f * range_max);
    lidar_info->scan_angle_min = scan->angle_min;
    lidar_info->scan_angle_increment = scan->angle_increment;
    lidar_info->scan_size = scan->ranges.size();
    // I am adding 50 floats for security since not all scans have the
    // same number of points and I only initialize the array for the points
    // in the first "turn" of the lidar
    const int scan_size_const = (lidar_info->scan_size + 50);
    scan_points = new float[scan_size_const];
    lidar_init = true;
  }

  // Iterate through laser scan ranges
  if (scan_points != nullptr)
    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      scan_points[i] = scan->ranges[i];
      if ((i > lidar_right_side_ray_start) && (i < lidar_right_side_ray_end))
      {
        if (std::isfinite(scan_points[i]))
          lidar_average_distance += scan_points[i];
      }
    }
  lidar_average_distance /= (lidar_right_side_ray_end - lidar_right_side_ray_start);
  // lidar_distances.push_back(lidar_average_distance);
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackImu(const std_msgs::msg::Float32MultiArray::SharedPtr imu_euler)
{
  yaw = imu_euler->data[0]; // IMU_Euler Data

  if (yaw_init == -1.0)
  {
    yaw_init = yaw;
  }
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackMotorTurns(const std_msgs::msg::Int32::SharedPtr msg)
{
  motor_turns = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::topic_callback_us_0(const std_msgs::msg::Float32::SharedPtr msg_us_0)
{
  us_0 = msg_us_0->data;
}
//----------------------------------------------------------------------------------------------------
void Parking::topic_callback_us_1(const std_msgs::msg::Float32::SharedPtr msg_us_1)
{
  us_1 = msg_us_1->data;
}
//----------------------------------------------------------------------------------------------------
void Parking::topic_callback_us_2(const std_msgs::msg::Float32::SharedPtr msg_us_2)
{
  us_2 = msg_us_2->data;
}
//----------------------------------------------------------------------------------------------------
void Parking::topic_callback_us_3(const std_msgs::msg::Float32::SharedPtr msg_us_3)
{
  us_3 = msg_us_3->data;
}
//----------------------------------------------------------------------------------------------------
void Parking::topic_callback_us_4(const std_msgs::msg::Float32::SharedPtr msg_us_4)
{
  us_4 = msg_us_4->data;
}
//----------------------------------------------------------------------------------------------------
void Parking::topic_callback_us_5(const std_msgs::msg::Float32::SharedPtr msg_us_5)
{
  us_5 = msg_us_5->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackParkingIn(const std_msgs::msg::Bool::SharedPtr msg)
{
  parking_in = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::callbackParkingOut(const std_msgs::msg::Bool::SharedPtr msg)
{
  parking_out = msg->data;
}
//----------------------------------------------------------------------------------------------------

void Parking::timerCallback()
{

  std_msgs::msg::Float32 parking_steering;
  std_msgs::msg::Float32 parking_speed;

  parking_motor_values.steering = parking_steer_straight;
  parking_motor_values.speed = parking_speed_stop;
  parking_steering.data = parking_steer_straight;
  parking_speed.data = parking_speed_stop;

  if (parking_maneuver == true)
  {
    parking_steering.data = navigation_steering;
    parking_speed.data = navigation_speed;

    if (parallel_parking == true) // Parallel Parking Case
    {
      if (parking_in == false)
        parallel_in_start = true;
      if (parking_out == false)
        parallel_out_start = true;

      // Parking in and out routine
      if (parking_in == true)
      { // PARKING IN PARALLEL PARKING
          CrossParkingIn::Ultrasound usdata;
          CrossParkingIn::Lidar lidardata;
          usdata.left = us_5;
          usdata.right = us_2;
          usdata.rear_left = us_4;
          usdata.rear_right = us_3;
          usdata.front_left = us_0;
          usdata.front_right = us_1;
          lidardata.angleincrement = lidar.angle_increment;
          lidardata.scan_time = lidar.scan_time;
          lidardata.time_increment = lidar.time_increment;
          lidardata.distances = lidar.ranges; 
          parallel_in.sensors->Update(usdata);
          parallel_in.sensors->Update(yaw);
          parallel_in.sensors->Update(motor_turns);
          parallel_in.sensors->Update(lidardata);
          switch (parallel_in.Update())
          {
          case ParallelParkingIn::ParallelParkingStates::FINISH:
            RCLCPP_INFO(this->get_logger(), "FINISH");
            break;
          case ParallelParkingIn::ParallelParkingStates::POSITION_FOUND:
            RCLCPP_INFO(this->get_logger(), "POSITION_FOUND");
            break;
          case ParallelParkingIn::ParallelParkingStates::START:
            RCLCPP_INFO(this->get_logger(), "START");
            break;
          case ParallelParkingIn::ParallelParkingStates::STOP:
            RCLCPP_INFO(this->get_logger(), "STOP");
            break;
          case ParallelParkingIn::ParallelParkingStates::FORWARD_STRAIGHT:
            RCLCPP_INFO(this->get_logger(), "FORWARD_STRAIGHT");
            break;
          case ParallelParkingIn::ParallelParkingStates::FORWARD_RIGHT:
            RCLCPP_INFO(this->get_logger(), "FORWARD_RIGHT");
            break;
          case ParallelParkingIn::ParallelParkingStates::REVERSE_RIGHT:
            RCLCPP_INFO(this->get_logger(), "REVERSE_RIGHT");
            break;
          case ParallelParkingIn::ParallelParkingStates::REVERSE_LEFT:
            RCLCPP_INFO(this->get_logger(), "REVERSE_LEFT");
            break;
          case ParallelParkingIn::ParallelParkingStates::REVERSE_STRAIGHT:
            RCLCPP_INFO(this->get_logger(), "REVERSE_STRAIGHT");
            break;
          case ParallelParkingIn::ParallelParkingStates::ENUM_LAST:
            RCLCPP_INFO(this->get_logger(), "ENUM_LAST");
            break;
          default: 
            RCLCPP_INFO(this->get_logger(), "DEFAULT");
            break;
          }
          parking_motor_values.speed = parallel_in.getParkingSpeed();
          parking_motor_values.steering = parallel_in.getParkingSteering();

      }
      else if (parking_out == true)
      { // PARKING OUT PARALLEL PARKING
      }

      parking_steering.data = parking_motor_values.steering;
      // parking_speed.data = navigation_speed;
      parking_speed.data = parking_motor_values.speed;
    }
    else if (cross_parking == true) // Cross Parking Case
    {
      if (parking_in == false)
        cross_in_start = true;
      if (parking_out == false)
        cross_out_start = true;

      if (parking_in == true)
      { // PARKING IN CROSS PARKING
        if (cross_in_start == true)
        {

          CrossParkingIn::Ultrasound usdata;
          CrossParkingIn::Lidar lidardata;
          usdata.left = us_5;
          usdata.right = us_2;
          usdata.rear_left = us_4;
          usdata.rear_right = us_3;
          usdata.front_left = us_0;
          usdata.front_right = us_1;
          lidardata.angleincrement = lidar.angle_increment;
          lidardata.scan_time = lidar.scan_time;
          lidardata.time_increment = lidar.time_increment;
          lidardata.distances = lidar.ranges; 
          cross_in.sensors->Update(usdata);
          cross_in.sensors->Update(yaw);
          cross_in.sensors->Update(motor_turns);
          cross_in.sensors->Update(lidardata);
          switch (cross_in.Update())
          {
          case CrossParkingIn::CrossParkingStates::FINISH:
            RCLCPP_INFO(this->get_logger(), "FINISH");
            break;
          case CrossParkingIn::CrossParkingStates::POSITION_FOUND:
            RCLCPP_INFO(this->get_logger(), "POSITION_FOUND");
            break;
          case CrossParkingIn::CrossParkingStates::START:
            RCLCPP_INFO(this->get_logger(), "START");
            break;
          case CrossParkingIn::CrossParkingStates::STOP:
            RCLCPP_INFO(this->get_logger(), "STOP");
            break;
          case CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT:
            RCLCPP_INFO(this->get_logger(), "FORWARD_STRAIGHT");
            break;
          case CrossParkingIn::CrossParkingStates::FORWARD_RIGHT:
            RCLCPP_INFO(this->get_logger(), "FORWARD_RIGHT");
            break;
          case CrossParkingIn::CrossParkingStates::FORWARD_LEFT:
            RCLCPP_INFO(this->get_logger(), "FORWARD_LEFT");
            break;
          case CrossParkingIn::CrossParkingStates::REVERSE_RIGHT:
            RCLCPP_INFO(this->get_logger(), "REVERSE_RIGHT");
            break;
          case CrossParkingIn::CrossParkingStates::REVERSE_LEFT:
            RCLCPP_INFO(this->get_logger(), "REVERSE_LEFT");
            break;
          case CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT:
            RCLCPP_INFO(this->get_logger(), "REVERSE_STRAIGHT");
            break;
          case CrossParkingIn::CrossParkingStates::ENUM_LAST:
            RCLCPP_INFO(this->get_logger(), "ENUM_LAST");
            break;
          }
          parking_motor_values.speed = cross_in.getParkingSpeed();
          parking_motor_values.steering = cross_in.getParkingSteering();

        }
        else
        {

          //  todo Hartmut Köhn  no navigation with Buggie

 /*          parking_motor_values.steering = navigation_steering;
          parking_motor_values.speed = car_speed_drive; */
        }
      }
      else if (parking_out == true)
      { // PARKING OUT CROSS PARKING
        parking_motor_values = cross.park_out(motor_turns, turns_to_cms, cross_out_advance, car_speed_drive,
                                              cross_out_turn_left, cross_out_advance_right, cross_out_turn_right, navigation_steering, cross_out_start,
                                              cross_out_advance_pit_out, parking_steer_straight, parking_speed_stop);
        cross_out_start = false;
      }
      // Parking in and out routine
      parking_steering.data = parking_motor_values.steering;
      // parking_speed.data = navigation_speed;
      parking_speed.data = parking_motor_values.speed;
      
      // steering_pub->publish(parking_steering);
      // motor_speed_pub->publish(parking_speed);
    }
  }
  /*__________Publish an image with the sensor measurements_________*/

    // Define laser image and image size
    /*
   cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC3);
   cv::Mat *image_address = &image;
   CreateSensorImage sensor_image;
   sensor_image.create_image(image_address, img_size, lidar_info, scan_points,
                             yaw, yaw_init, parking_params, us_dist, lidar_right_side_angle,
                             lidar_right_side_ray_start, lidar_right_side_ray_end);

   // Convert image to ROS message
   std_msgs::msg::Header header;
   header.stamp = this->now();
   cv_bridge::CvImage cv_image(header, "bgr8", image);

   // Convert the image to a ROS2 message
   auto msg = cv_image.toImageMsg();
   publisher_image->publish(*msg); 
   */
  steering_pub->publish(parking_steering);
  motor_speed_pub->publish(parking_speed);
}
//----------------------------------------------------------------------------------------------------

bool Parking::free_box_found()
{
  bool free_box = false;

  if ((lidar_average_distance * 100 > dist_to_start_of_box) &&
      (lidar_average_distance * 100 < dist_to_end_of_box))
    free_box_length++;
  else if ((lidar_average_distance * 100 <= dist_to_start_of_box) ||
           (lidar_average_distance * 100 >= dist_to_end_of_box))
  {
    free_box_length = 0;
  }
  if (free_box_length > box_length)
    free_box = true;

  return free_box;
}
//----------------------------------------------------------------------------------------------------

void Parking::initMotorTurns(int distInCms)
{
  motor_turns_init = motor_turns;
  motor_turns_should = (distInCms / turns_to_cms);
}
//----------------------------------------------------------------------------------------------------

void Parking::readParametersFromYaml()
{
  // Declare parameters for test driving a fixed segment
  this->declare_parameter<int>("dist_cms_p", 300);
  this->declare_parameter<double>("car_speed_drive_p", 1.017);
  this->declare_parameter<double>("car_speed_stop_p", 1.0);
  this->declare_parameter<double>("turns_to_cms_p", 3.85);

  // Declare parameters for Parking
  this->declare_parameter<double>("parking_steer_straight_p", 1.0);
  this->declare_parameter<double>("parking_speed_stop_p", 1.0);

  // Declare parameters for Cross Parking
  this->declare_parameter<int>("cross_out_advance_p", 0);
  this->declare_parameter<int>("cross_out_advance_right_p", 0);
  this->declare_parameter<int>("cross_out_advance_pit_out_p", 0);
  this->declare_parameter<double>("cross_out_turn_left_p", 1.4);
  this->declare_parameter<double>("cross_out_turn_right_p", 0.6);
  this->declare_parameter<int>("cross_imu_turn_detected_p", 0);
  // this->declare_parameter<double>("cross_in_advance_p", 1000);
  this->declare_parameter<int>("dist_to_start_of_box_p", 50);
  this->declare_parameter<int>("dist_to_end_of_box_p", 120);
  this->declare_parameter<int>("box_length_p", 50);

  // Declare parameters for Parallel Parking
  this->declare_parameter<int>("pit_in_parallel_advance_p", 220);

  // Declare the parameters for reading the Lidar's rays of interest
  this->declare_parameter<double>("lidar_right_side_angle_p", 1.5707);
  this->declare_parameter<double>("lidar_right_side_angle_min_p", 2.2686);
  this->declare_parameter<double>("lidar_right_side_angle_max_p", 2.4422);
  this->declare_parameter<int>("lidar_right_side_ray_start_p", 402);
  this->declare_parameter<int>("lidar_right_side_ray_end_p", 412);

  // Get parameters from the parameter server
  dist_cms = this->get_parameter("dist_cms_p").as_int();
  car_speed_drive = this->get_parameter("car_speed_drive_p").as_double();
  car_speed_stop = this->get_parameter("car_speed_stop_p").as_double();
  turns_to_cms = this->get_parameter("turns_to_cms_p").as_double();

  parking_steer_straight = this->get_parameter("parking_steer_straight_p").as_double();
  parking_speed_stop = this->get_parameter("parking_speed_stop_p").as_double();

  cross_out_advance = this->get_parameter("cross_out_advance_p").as_int();
  cross_out_advance_right = this->get_parameter("cross_out_advance_right_p").as_int();
  cross_out_advance_pit_out = this->get_parameter("cross_out_advance_pit_out_p").as_int();
  cross_out_turn_left = this->get_parameter("cross_out_turn_left_p").as_double();
  cross_out_turn_right = this->get_parameter("cross_out_turn_right_p").as_double();
  cross_imu_turn_detected = this->get_parameter("cross_imu_turn_detected_p").as_int();
  // cross_in_advance = this->get_parameter("cross_in_advance_p").as_double();
  dist_to_start_of_box = this->get_parameter("dist_to_start_of_box_p").as_int();
  dist_to_end_of_box = this->get_parameter("dist_to_end_of_box_p").as_int();
  box_length = this->get_parameter("box_length_p").as_int();

  pit_in_parallel_advance = this->get_parameter("pit_in_parallel_advance_p").as_int();

  lidar_right_side_angle = this->get_parameter("lidar_right_side_angle_p").as_double();
  lidar_right_side_angle_min = this->get_parameter("lidar_right_side_angle_min_p").as_double();
  lidar_right_side_angle_max = this->get_parameter("lidar_right_side_angle_max_p").as_double();
  lidar_right_side_ray_start = this->get_parameter("lidar_right_side_ray_start_p").as_int();
  lidar_right_side_ray_end = this->get_parameter("lidar_right_side_ray_end_p").as_int();

  RCLCPP_INFO(this->get_logger(), "cross_out_turn_right: %f", cross_out_turn_right);
  RCLCPP_INFO(this->get_logger(), "cross_out_advance: %d", cross_out_advance);
}
//----------------------------------------------------------------------------------------------------

//____________________________________________________________________________________________________
//____________________________________________________________________________________________________
//____________________________________________________________________________________________________

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Parking>());
  rclcpp::shutdown();

  return (0);
}
