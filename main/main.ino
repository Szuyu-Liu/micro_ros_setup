#include "CarController.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

CarController car;
String command;

rcl_subscription_t speed_sub, distance_sub, direction_sub;
std_msgs__msg__Float32 msgspeed;
std_msgs__msg__Float32 msgdistance;
std_msgs__msg__Float32 msgdirection;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void speed_subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msgspeed = (const std_msgs__msg__Float32 *)msgin;
  command = command+"speed:"+String(msgspeed->data);
}

void distance_subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msgdistance = (const std_msgs__msg__Float32 *)msgin;
  command = command+"distance:"+String(msgdistance->data);
}

void direction_subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * msgdirection = (const std_msgs__msg__Float32 *)msgin;
  command = command+"direction:"+String(msgdirection->data);
}

void setup() {

  car.initialize();

  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support);

   // create subscriber
  rclc_subscription_init_default(
    &speed_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "car/speed");

  rclc_subscription_init_default(
    &distance_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "car/distance");
  
  rclc_subscription_init_default(
    &direction_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "car/direction");

  // create executor
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &speed_sub, &msgspeed, &speed_subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &distance_sub, &msgdistance, &distance_subscription_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &direction_sub, &msgdirection, &direction_subscription_callback, ON_NEW_DATA);
}

void loop() {
  car.run(command);
  command = "";
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}