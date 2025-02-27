#include "parking/parking.hpp"

void Parking::readParametersCrossInFromYaml(vector<CrossParkingIn::TransitionConditions> &states,
                                            CrossParkingIn::MaxValuesCrossParking &maxvalues)
{
    //  # max/min values
    this->declare_parameter<double>("ci_steering_straight_p", 1.0);
    this->declare_parameter<double>("ci_steering_left_p", 0.0);
    this->declare_parameter<double>("ci_steering_right_p", 2.0);
    this->declare_parameter<double>("ci_speed_stop_p", 1.0);
    this->declare_parameter<double>("ci_speed_forward_p", 1.20);
    this->declare_parameter<double>("ci_speed_reverse_p", 0.80);
    this->declare_parameter<double>("ci_distance_forward_single_step_p", 100);
    this->declare_parameter<double>("ci_distance_reverse_single_step_p", 100);
    this->declare_parameter<double>("ci_distance_reverse_p", 5000);
    this->declare_parameter<double>("ci_distance_forward_p", 5000);
    this->declare_parameter<double>("ci_lidar_angle_offset_p", 0.0);
    this->declare_parameter<double>("ci_wall_distance_p", 0.0);

    //    # all values are absolute
    //  # conditions to perform forward left correction move at start //multi move maneuver
    this->declare_parameter<double>("FORWARD_LEFT_imu_min_p", -1.);
    this->declare_parameter<double>("FORWARD_LEFT_imu_max_p", 90.);
    this->declare_parameter<double>("FORWARD_LEFT_lidar_p", 0.);
    this->declare_parameter<double>("FORWARD_LEFT_us_left_p", -1.);
    this->declare_parameter<double>("FORWARD_LEFT_us_right_p", 50.);
    this->declare_parameter<double>("FORWARD_LEFT_us_rear_left_p", -1.);
    this->declare_parameter<double>("FORWARD_LEFT_us_rear_right_p", -1.);
    this->declare_parameter<double>("FORWARD_LEFT_us_front_left_p", -1.);
    this->declare_parameter<double>("FORWARD_LEFT_us_front_right_p", -1.);
    this->declare_parameter<int>("FORWARD_LEFT_previous_state_p", -1);
    this->declare_parameter<double>("FORWARD_LEFT_distance_p", 400);

    //  # conditions to perform forward right correction move anytime
    this->declare_parameter<double>("FORWARD_RIGHT_imu_min_p", -10);
    this->declare_parameter<double>("FORWARD_RIGHT_imu_max_p", 10);
    this->declare_parameter<double>("FORWARD_RIGHT_lidar_p", 0);
    this->declare_parameter<double>("FORWARD_RIGHT_us_left_p", -1);
    this->declare_parameter<double>("FORWARD_RIGHT_us_right_p", 400);
    this->declare_parameter<double>("FORWARD_RIGHT_us_rear_left_p", -1);
    this->declare_parameter<double>("FORWARD_RIGHT_us_rear_right_p", -1);
    this->declare_parameter<double>("FORWARD_RIGHT_us_front_left_p", -1);
    this->declare_parameter<double>("FORWARD_RIGHT_us_front_right_p", -1);
    this->declare_parameter<int>("FORWARD_RIGHT_previous_state_p", 6);
    this->declare_parameter<double>("FORWARD_RIGHT_distance_p", 200);

    //  # conditions to perform reverse right correction move anytime
    this->declare_parameter<double>("REVERSE_RIGHT_imu_min_p", -10);
    this->declare_parameter<double>("REVERSE_RIGHT_imu_max_p", 10);
    this->declare_parameter<double>("REVERSE_RIGHT_lidar_p", 0);
    this->declare_parameter<double>("REVERSE_RIGHT_us_left_p", -1);
    this->declare_parameter<double>("REVERSE_RIGHT_us_right_p", 400);
    this->declare_parameter<double>("REVERSE_RIGHT_us_rear_left_p", -1);
    this->declare_parameter<double>("REVERSE_RIGHT_us_rear_right_p", -1);
    this->declare_parameter<double>("REVERSE_RIGHT_us_front_left_p", -1);
    this->declare_parameter<double>("REVERSE_RIGHT_us_front_right_p", -1);
    this->declare_parameter<int>("REVERSE_RIGHT_previous_state_p", 6);
    this->declare_parameter<double>("REVERSE_RIGHT_distance_p", 200);

    //  # conditions to perform reverse left correction move anytime
    this->declare_parameter<double>("REVERSE_LEFT_imu_min_p", -10);
    this->declare_parameter<double>("REVERSE_LEFT_imu_max_p", 10);
    this->declare_parameter<double>("REVERSE_LEFT_lidar_p", 0);
    this->declare_parameter<double>("REVERSE_LEFT_us_left_p", -1);
    this->declare_parameter<double>("REVERSE_LEFT_us_right_p", 400);
    this->declare_parameter<double>("REVERSE_LEFT_us_rear_left_p", -1);
    this->declare_parameter<double>("REVERSE_LEFT_us_rear_right_p", -1);
    this->declare_parameter<double>("REVERSE_LEFT_us_front_left_p", -1);
    this->declare_parameter<double>("REVERSE_LEFT_us_front_right_p", -1);
    this->declare_parameter<int>("REVERSE_LEFT_previous_state_p", 6);
    this->declare_parameter<double>("REVERSE_LEFT_distance_p", 200);

    //  # conditions to perform reverse straight correction move anytime
    this->declare_parameter<double>("REVERSE_STRAIGHT_imu_min_p", -10);
    this->declare_parameter<double>("REVERSE_STRAIGHT_imu_max_p", 10);
    this->declare_parameter<double>("REVERSE_STRAIGHT_lidar_p", 0);
    this->declare_parameter<double>("REVERSE_STRAIGHT_us_left_p", -1);
    this->declare_parameter<double>("REVERSE_STRAIGHT_us_right_p", 400);
    this->declare_parameter<double>("REVERSE_STRAIGHT_us_rear_left_p", -1);
    this->declare_parameter<double>("REVERSE_STRAIGHT_us_rear_right_p", -1);
    this->declare_parameter<double>("REVERSE_STRAIGHT_us_front_left_p", -1);
    this->declare_parameter<double>("REVERSE_STRAIGHT_us_front_right_p", -1);
    this->declare_parameter<int>("REVERSE_STRAIGHT_previous_state_p", 6);
    this->declare_parameter<double>("REVERSE_STRAIGHT_distance_p", 50);

    //  # conditions to perform forward straight correction move anytime
    this->declare_parameter<double>("FORWARD_STRAIGHT_imu_min_p", -10);
    this->declare_parameter<double>("FORWARD_STRAIGHT_imu_max_p", 10);
    this->declare_parameter<double>("FORWARD_STRAIGHT_lidar_p", 0);
    this->declare_parameter<double>("FORWARD_STRAIGHT_us_left_p", -1);
    this->declare_parameter<double>("FORWARD_STRAIGHT_us_right_p", 400);
    this->declare_parameter<double>("FORWARD_STRAIGHT_us_rear_left_p", -1);
    this->declare_parameter<double>("FORWARD_STRAIGHT_us_rear_right_p", -1);
    this->declare_parameter<double>("FORWARD_STRAIGHT_us_front_left_p", -1);
    this->declare_parameter<double>("FORWARD_STRAIGHT_us_front_right_p", -1);
    this->declare_parameter<int>("FORWARD_STRAIGHT_previous_state_p", 6);
    this->declare_parameter<double>("FORWARD_STRAIGHT_distance_p", 50);

        // # conditions for start
    this->declare_parameter<double>("START_imu_min_p", -1);
    this->declare_parameter<double>("START_imu_max_p", -1);
    this->declare_parameter<double>("START_lidar_p", -1);
    this->declare_parameter<double>("START_us_left_p", -1);
    this->declare_parameter<double>("START_us_right_p", -1);
    this->declare_parameter<double>("START_us_rear_left_p", .5);
    this->declare_parameter<double>("START_us_rear_right_p", .5);
    this->declare_parameter<double>("START_us_front_left_p", .5);
    this->declare_parameter<double>("START_us_front_right_p", .5);
    this->declare_parameter<int>("START_previous_state_p", -1);
    this->declare_parameter<double>("START_distance_p", -0.1);

    // # conditions for stop
    this->declare_parameter<double>("STOP_imu_min_p", -1);
    this->declare_parameter<double>("STOP_imu_max_p", -1);
    this->declare_parameter<double>("STOP_lidar_p", -1);
    this->declare_parameter<double>("STOP_us_left_p", 30);
    this->declare_parameter<double>("STOP_us_right_p", 30);
    this->declare_parameter<double>("STOP_us_rear_left_p", 50);
    this->declare_parameter<double>("STOP_us_rear_right_p", 50);
    this->declare_parameter<double>("STOP_us_front_left_p", 50);
    this->declare_parameter<double>("STOP_us_front_right_p", 50);
    this->declare_parameter<int>("STOP_previous_state_p", -1);
    this->declare_parameter<double>("STOP_distance_p", -1);

    // # conditions for finish
    this->declare_parameter<double>("FINISH_imu_min_p", 85);
    this->declare_parameter<double>("FINISH_imu_max_p", 95);
    this->declare_parameter<double>("FINISH_lidar_p", -1);
    this->declare_parameter<double>("FINISH_us_left_p", 50);
    this->declare_parameter<double>("FINISH_us_right_p", 50);
    this->declare_parameter<double>("FINISH_us_rear_left_p", 50);
    this->declare_parameter<double>("FINISH_us_rear_right_p", 50);
    this->declare_parameter<double>("FINISH_us_front_left_p", -1);
    this->declare_parameter<double>("FINISH_us_front_right_p", -1);
    this->declare_parameter<int>("FINISH_previous_state_p", -1);
    this->declare_parameter<double>("FINISH_distance_p", -1);


       
    maxvalues.steering_straight = this->get_parameter("ci_steering_straight_p").as_double();
    maxvalues.steering_left = this->get_parameter("ci_steering_left_p").as_double();
    maxvalues.steering_right = this->get_parameter("ci_steering_right_p").as_double();
    maxvalues.speed_stop = this->get_parameter("ci_speed_stop_p").as_double();
    maxvalues.speed_forward = this->get_parameter("ci_speed_forward_p").as_double();
    maxvalues.speed_reverse = this->get_parameter("ci_speed_reverse_p").as_double();
    maxvalues.distance_forward_single_step = this->get_parameter("ci_distance_forward_single_step_p").as_double();
    maxvalues.distance_reverse_single_step = this->get_parameter("ci_distance_reverse_single_step_p").as_double();
    maxvalues.distance_reverse = this->get_parameter("ci_distance_reverse_p").as_double();
    maxvalues.distance_forward = this->get_parameter("ci_distance_forward_p").as_double(); 
    maxvalues.lidar_correction.lidar_angle_offset = this->get_parameter("ci_lidar_angle_offset_p").as_double();
    maxvalues.lidar_correction.wall_distance = this->get_parameter("ci_wall_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].imu_min = this->get_parameter("FORWARD_LEFT_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].imu_max = this->get_parameter("FORWARD_LEFT_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].lidar = this->get_parameter("FORWARD_LEFT_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].us_left = this->get_parameter("FORWARD_LEFT_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].us_right = this->get_parameter("FORWARD_LEFT_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].us_rear_left = this->get_parameter("FORWARD_LEFT_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].us_rear_right = this->get_parameter("FORWARD_LEFT_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].us_front_left = this->get_parameter("FORWARD_LEFT_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].us_front_right = this->get_parameter("FORWARD_LEFT_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].previous_state = this->get_parameter("FORWARD_LEFT_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::FORWARD_LEFT].distance = this->get_parameter("FORWARD_LEFT_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].imu_min = this->get_parameter("REVERSE_RIGHT_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].imu_max = this->get_parameter("REVERSE_RIGHT_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].lidar = this->get_parameter("REVERSE_RIGHT_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].us_left = this->get_parameter("REVERSE_RIGHT_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].us_right = this->get_parameter("REVERSE_RIGHT_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].us_rear_left = this->get_parameter("REVERSE_RIGHT_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].us_rear_right = this->get_parameter("REVERSE_RIGHT_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].us_front_left = this->get_parameter("REVERSE_RIGHT_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].us_front_right = this->get_parameter("REVERSE_RIGHT_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].previous_state = this->get_parameter("REVERSE_RIGHT_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::REVERSE_RIGHT].distance = this->get_parameter("REVERSE_RIGHT_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].imu_min = this->get_parameter("FORWARD_RIGHT_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].imu_max = this->get_parameter("FORWARD_RIGHT_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].lidar = this->get_parameter("FORWARD_RIGHT_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].us_left = this->get_parameter("FORWARD_RIGHT_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].us_right = this->get_parameter("FORWARD_RIGHT_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].us_rear_left = this->get_parameter("FORWARD_RIGHT_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].us_rear_right = this->get_parameter("FORWARD_RIGHT_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].us_front_left = this->get_parameter("FORWARD_RIGHT_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].us_front_right = this->get_parameter("FORWARD_RIGHT_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].previous_state = this->get_parameter("FORWARD_RIGHT_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::FORWARD_RIGHT].distance = this->get_parameter("FORWARD_RIGHT_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].imu_min = this->get_parameter("REVERSE_LEFT_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].imu_max = this->get_parameter("REVERSE_LEFT_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].lidar = this->get_parameter("REVERSE_LEFT_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].us_left = this->get_parameter("REVERSE_LEFT_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].us_right = this->get_parameter("REVERSE_LEFT_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].us_rear_left = this->get_parameter("REVERSE_LEFT_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].us_rear_right = this->get_parameter("REVERSE_LEFT_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].us_front_left = this->get_parameter("REVERSE_LEFT_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].us_front_right = this->get_parameter("REVERSE_LEFT_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].previous_state = this->get_parameter("REVERSE_LEFT_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::REVERSE_LEFT].distance = this->get_parameter("REVERSE_LEFT_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].imu_min = this->get_parameter("REVERSE_STRAIGHT_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].imu_max = this->get_parameter("REVERSE_STRAIGHT_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].lidar = this->get_parameter("REVERSE_STRAIGHT_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].us_left = this->get_parameter("REVERSE_STRAIGHT_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].us_right = this->get_parameter("REVERSE_STRAIGHT_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].us_rear_left = this->get_parameter("REVERSE_STRAIGHT_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].us_rear_right = this->get_parameter("REVERSE_STRAIGHT_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].us_front_left = this->get_parameter("REVERSE_STRAIGHT_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].us_front_right = this->get_parameter("REVERSE_STRAIGHT_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].previous_state = this->get_parameter("REVERSE_STRAIGHT_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::REVERSE_STRAIGHT].distance = this->get_parameter("REVERSE_STRAIGHT_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].imu_min = this->get_parameter("FORWARD_STRAIGHT_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].imu_max = this->get_parameter("FORWARD_STRAIGHT_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].lidar = this->get_parameter("FORWARD_STRAIGHT_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].us_left = this->get_parameter("FORWARD_STRAIGHT_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].us_right = this->get_parameter("FORWARD_STRAIGHT_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].us_rear_left = this->get_parameter("FORWARD_STRAIGHT_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].us_rear_right = this->get_parameter("FORWARD_STRAIGHT_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].us_front_left = this->get_parameter("FORWARD_STRAIGHT_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].us_front_right = this->get_parameter("FORWARD_STRAIGHT_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].previous_state = this->get_parameter("FORWARD_STRAIGHT_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::FORWARD_STRAIGHT].distance = this->get_parameter("FORWARD_STRAIGHT_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::START].imu_min = this->get_parameter("START_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].imu_max = this->get_parameter("START_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].lidar = this->get_parameter("START_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].us_left = this->get_parameter("START_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].us_right = this->get_parameter("START_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].us_rear_left = this->get_parameter("START_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].us_rear_right = this->get_parameter("START_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].us_front_left = this->get_parameter("START_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].us_front_right = this->get_parameter("START_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::START].previous_state = this->get_parameter("START_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::START].distance = this->get_parameter("START_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::STOP].imu_min = this->get_parameter("STOP_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].imu_max = this->get_parameter("STOP_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].lidar = this->get_parameter("STOP_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].us_left = this->get_parameter("STOP_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].us_right = this->get_parameter("STOP_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].us_rear_left = this->get_parameter("STOP_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].us_rear_right = this->get_parameter("STOP_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].us_front_left = this->get_parameter("STOP_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].us_front_right = this->get_parameter("STOP_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::STOP].previous_state = this->get_parameter("STOP_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::STOP].distance = this->get_parameter("STOP_distance_p").as_double();

    states[CrossParkingIn::CrossParkingStates::FINISH].imu_min = this->get_parameter("FINISH_imu_min_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].imu_max = this->get_parameter("FINISH_imu_max_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].lidar = this->get_parameter("FINISH_lidar_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].us_left = this->get_parameter("FINISH_us_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].us_right = this->get_parameter("FINISH_us_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].us_rear_left = this->get_parameter("FINISH_us_rear_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].us_rear_right = this->get_parameter("FINISH_us_rear_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].us_front_left = this->get_parameter("FINISH_us_front_left_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].us_front_right = this->get_parameter("FINISH_us_front_right_p").as_double();
    states[CrossParkingIn::CrossParkingStates::FINISH].previous_state = this->get_parameter("FINISH_previous_state_p").as_int();
    states[CrossParkingIn::CrossParkingStates::FINISH].distance = this->get_parameter("FINISH_distance_p").as_double();
}