#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include "parking/parking_datatypes.hpp"
#include "parking/create_sensor_image.hpp"
#include "parking/cross_parking_1.hpp"

#include "cross_in/cross_parking_statemachine.h"
#include "parallel_in/parallel_parking_statemachine.h"

#define RAD2DEG(x) ((x) * 180. / M_PI)

class Parking : public rclcpp::Node
{
public:
    Parking();

private:
    void readParametersFromYaml();
    void readParametersCrossInFromYaml(vector<CrossParkingIn::TransitionConditions> &, CrossParkingIn::MaxValuesCrossParking &);
    void readParametersParallelInFromYaml(vector< ParallelParkingIn::TransitionConditions> &states,
                                            ParallelParkingIn::MaxValuesParallelParking &maxvalues);
    void initMotorTurns(int);
    bool free_box_found();
    // Callback functions
    void callbackMotorTurns(const std_msgs::msg::Int32::SharedPtr msg);
    void callbackGUIMode(const std_msgs::msg::Int32::SharedPtr msg);
    void callbackCrossParking(const std_msgs::msg::Bool::SharedPtr msg);
    void callbackParallelParking(const std_msgs::msg::Bool::SharedPtr msg);
    void callbackGUIParkingManeuver(const std_msgs::msg::Bool::SharedPtr msg);
    void callbackParkingIn(const std_msgs::msg::Bool::SharedPtr msg);
    void callbackParkingOut(const std_msgs::msg::Bool::SharedPtr msg);

    // Navigation
    void callbackCAMSpeed(const std_msgs::msg::Float32::SharedPtr msg);
    void callbackCAMSteering(const std_msgs::msg::Float32::SharedPtr msg);
    
    // US sensors
    void topic_callback_us_0(const std_msgs::msg::Float32::SharedPtr msg_us_0);
    void topic_callback_us_1(const std_msgs::msg::Float32::SharedPtr msg_us_1);
    void topic_callback_us_2(const std_msgs::msg::Float32::SharedPtr msg_us_2);
    void topic_callback_us_3(const std_msgs::msg::Float32::SharedPtr msg_us_3);
    void topic_callback_us_4(const std_msgs::msg::Float32::SharedPtr msg_us_4);
    void topic_callback_us_5(const std_msgs::msg::Float32::SharedPtr msg_us_5);

    void callbackLidarScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void callbackImu(const std_msgs::msg::Float32MultiArray::SharedPtr imu_euler);

    void timerCallback();

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_motorcontroller_turns;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_gui_mode;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gui_parallel_parking;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gui_cross_parking;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gui_parking_maneuver;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gui_parking_in;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_gui_parking_out;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cam_speed_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_cam_steering_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_nn_speed_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_nn_steering_;

    // US sensors
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_us_0;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_us_1;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_us_2;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_us_3;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_us_4;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_us_5;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_speed_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image;
    // Timer
    rclcpp::TimerBase::SharedPtr timer;

    // State variables
    int autonomous_mode;
    bool cross_parking, parallel_parking, parking_maneuver, parking_in, parking_out;
    float navigation_speed, navigation_steering;
    float us_0, us_1, us_2, us_3, us_4, us_5;

    // Global Variables
    CrossParking cross;
    ParkingActions parking_mode = parking_end;
    ParkingMotorValues parking_motor_values;

    CrossParkingIn::Statemachine cross_in;
    ParallelParkingIn::Statemachine parallel_in;

    sensor_msgs::msg::LaserScan lidar;
    bool first_speed = true;
    float yaw = 1.0;
    float yaw_init = 1.0;
    int motor_turns = 0;
    int motor_turns_init = 0;
    float motor_turns_should = 0;
    int img_size = 400;
    std::vector<double> parking_params;
    int counter = 0;
    float us_dist[6]; 
    float camera_steering = 1.0;
    float cross_steering = 1.0;
    float parallel_steering = 1.0;
    float car_speed = 1.0;
    std::vector<double> lidar_distances;
    float lidar_average_distance = 0.0;
    int free_box_length = 0;

    /* Global flags to initialize sensors*/
    bool lidar_init = false;
    bool imu_init = false;
    bool motor_is_init = false;
    bool parallel_out_start = false;
    bool parallel_in_start = false;
    bool cross_out_start = false;
    bool cross_in_start = false;
    /* Global Pointers */
    float* scan_points = nullptr;
    LidarInfo* lidar_info = nullptr;
  
    /* Parameters read in from the config/parking_params.yaml file */
    double turns_to_cms = 3.85/4; //4 Signals per Turn
    double car_speed_drive = 1.01;
    double car_speed_stop = 1.0;
    double parking_speed_stop = 1.0;
    double parking_steer_straight = 1.0;
    int cross_out_advance = 0;
    int cross_out_advance_right = 0;
    int cross_out_advance_pit_out = 300;
    double cross_out_turn_left = 0.0;
    double cross_out_turn_right = 0.0;
    int cross_imu_turn_detected = 0.0;
    int dist_cms = 500;
    double lidar_right_side_angle = 1.5707; //90 degrees in radians
    double lidar_right_side_angle_min = 2.2686; //130 degrees in radians
    double lidar_right_side_angle_max = 2.4422; //140 degrees in radians
    int lidar_right_side_ray_start = 402; //ray corresponding to 130 degrees right side
    int lidar_right_side_ray_end = 412; //lidar ray corresponding to 140 degrees
    int pit_in_parallel_advance = 220; //analyze 2.2 meters for a parking spot
    double cross_in_advance = 1000;
    int dist_to_start_of_box = 50;
    int dist_to_end_of_box = 120;
    int box_length = 50;
   
};
