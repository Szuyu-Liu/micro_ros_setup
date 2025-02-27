#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <vector>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <math.h>

using namespace std;
using std::placeholders::_1;

#define RAD2DEG(x) ((x) * 180. / M_PI)
	
class CrossParking {
public:

  int cross_out_state_counter = 0;
  int cross_in_state_counter = 0;
  bool motor_reset_turn_counter = false;
  int motor_turns_init = 0;
  int motor_turns_should = 0;
  float cross_speed = 1.0;
  float cross_steering = 1.0;
  ParkingMotorValues cross_motor_values;
  
  CrossParking() { 
    resetCrossParkingStates();
  }
  
  ParkingMotorValues park_out(int motor_turns, float turns_to_cms, int cross_out_advance, float car_speed_drive,
    float cross_out_turn_left, float cross_out_advance_right, float cross_out_turn_right, float navigation_steering, 
    bool cross_out_start, int cross_out_advance_pit_out, float parking_steer_straight,float parking_speed_stop) 
    {
      if (cross_out_start == true) cross_out_state_counter = 0;
      
      switch (cross_out_state_counter) {
        case 0:
          //Read the number of turns of the motor at the start of the routine  
          if (motor_reset_turn_counter == false) {
            initMotorTurns(motor_turns, turns_to_cms, cross_out_advance);
            motor_reset_turn_counter = true;
          }
          //First action of cross_park_out: Drive straight 
          if(motor_turns_should > (abs(motor_turns - motor_turns_init))){
            cross_motor_values.speed = car_speed_drive;
            cross_motor_values.steering = parking_steer_straight;     				
          }
          else { 
            cross_out_state_counter++;
            motor_reset_turn_counter = false;
          }
          break;
        case 1:
          if (motor_reset_turn_counter == false){
              initMotorTurns(motor_turns, turns_to_cms, cross_out_advance_right);
              motor_reset_turn_counter = true;
          }
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))){
              cross_motor_values.speed = car_speed_drive;
              cross_motor_values.steering = cross_out_turn_right;  
          }
          else {
              cross_out_state_counter++;
              motor_reset_turn_counter = false;
          }
          break;
        case 2:
          if (motor_reset_turn_counter == false){
              initMotorTurns(motor_turns, turns_to_cms, cross_out_advance_pit_out);
              motor_reset_turn_counter = true;
          }
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))){
              cross_motor_values.speed = car_speed_drive;
              cross_motor_values.steering = navigation_steering;  
          }
          else {
              cross_out_state_counter++;
              motor_reset_turn_counter = false;
          }
          break;
        case 3:
            cross_motor_values.speed = parking_speed_stop;
  	        cross_motor_values.steering = parking_steer_straight;
  	    break; 
  	default:
  	    cross_motor_values.speed = parking_speed_stop;
  	    cross_motor_values.steering = parking_steer_straight;
  	  break;
        } 
      return cross_motor_values;
  }	
  
/*   ParkingMotorValues park_in(int motor_turns, float turns_to_cms, float car_speed_drive,
    float navigation_steering, float parking_steer_straight, float parking_speed_stop, 
    int cross_in_advance, bool cross_in_start)
  {
    if (cross_in_start == true) cross_in_state_counter = 0;
      
      switch (cross_in_state_counter) {
        case 0:
          //Read the number of turns of the motor at the start of the routine  
          if (motor_reset_turn_counter == false) {
            initMotorTurns(motor_turns, turns_to_cms, cross_in_advance);
            motor_reset_turn_counter = true;
          }
          //First action of cross_park_in: Find an empty box 
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))) {
           // cross_motor_values.speed = car_speed_drive;
            cross_motor_values.speed = parking_speed_stop;
            cross_motor_values.steering = parking_steer_straight;     				
          }
          else { 
            cross_in_state_counter++;
            motor_reset_turn_counter = false;
          }
          break; 
        case 1:
          if (motor_reset_turn_counter == false){
              initMotorTurns(motor_turns, turns_to_cms, cross_out_advance_right);
              motor_reset_turn_counter = true;
          }
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))){
              cross_motor_values.speed = car_speed_drive;
              cross_motor_values.steering = cross_out_turn_right;  
          }
          else {
              cross_out_state_counter++;
              motor_reset_turn_counter = false;
          }
          break;
        case 2:
          if (motor_reset_turn_counter == false){
              initMotorTurns(motor_turns, turns_to_cms, cross_out_advance_pit_out);
              motor_reset_turn_counter = true;
          }
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))){
              cross_motor_values.speed = car_speed_drive;
              cross_motor_values.steering = navigation_steering;  
          }
          else {
              //cross_park_out_states[1] = true;
              cross_out_state_counter++;
              motor_reset_turn_counter = false;
          }
          break;
        case 3:
            cross_motor_values.speed = parking_speed_stop;
  	        cross_motor_values.steering = parking_steer_straight;
  	    break; 
  	default:
  	    cross_motor_values.speed = parking_speed_stop;
  	    cross_motor_values.steering = parking_steer_straight;
  	  break;
        } 
      return cross_motor_values;
            return cross_motor_values;
            }

  ParkingMotorValues park_in(int motor_turns, float car_speed_drive,
    float cross_turn_left, float cross_turn_right, int cross_in_advance,
    bool cross_in_start, float parking_steer_straight)
  { 
        if (cross_in_start == true) cross_in_state_counter = 0;
      
      switch (cross_in_state_counter) {
        case 0:
          //Read the number of turns of the motor at the start of the routine  
          if (motor_reset_turn_counter == false) {
            motor_turns_init = motor_turns;
          }
          //First action 
          if(cross_in_advance > (abs(motor_turns - motor_turns_init))){
           // cross_motor_values.speed = car_speed_drive;
           // cross_motor_values.steering = cross_in_turn_left;     				
          }
          else { 
            cross_in_state_counter++;
            motor_reset_turn_counter = false;
          }
          break;
        case 1:
          if (motor_reset_turn_counter == false){
              motor_turns_init = motor_turns;
              motor_reset_turn_counter = true;
          }
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))){
             // cross_motor_values.speed = car_speed_drive;
             // cross_motor_values.steering = cross_out_turn_right;  
          }
          else {
              cross_in_state_counter++;
              motor_reset_turn_counter = false;
          }
          break;
        case 2:
          if (motor_reset_turn_counter == false){
              motor_turns_init = motor_turns;
              motor_reset_turn_counter = true;
          }
          if (motor_turns_should > (abs(motor_turns - motor_turns_init))){
             // cross_motor_values.speed = car_speed_drive;
             // cross_motor_values.steering = navigation_steering;  
          }
          else {
              //cross_park_out_states[1] = true;
              cross_in_state_counter++;
              motor_reset_turn_counter = false;
          }
          break;
        case 3:
            cross_motor_values.speed = parking_speed_stop;
  	        cross_motor_values.steering = parking_steer_straight;
  	    break; 
  	default:
  	    cross_motor_values.speed = parking_speed_stop;
  	    cross_motor_values.steering = parking_steer_straight;
  	  break;
        } 
      return cross_motor_values;  

  }*/

  void initMotorTurns(int motor_turns, double turns_to_cms, int distInCms){
    motor_turns_init = motor_turns;
    motor_turns_should = (distInCms/turns_to_cms);
  }
  void resetCrossParkingStates(){
    cross_out_state_counter = 0;
   //for (int i = 0; i < 4; i++)
     // cross_park_out_states[i] = false;
  }
  
};
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
