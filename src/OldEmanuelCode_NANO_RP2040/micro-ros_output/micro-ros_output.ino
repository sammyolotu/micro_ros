#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

//#include <input_messages/msg/InputStatus.h>
//#include <input_messages/msg/InputStatusArray.h>
//#include diegetic_button_pkg.msg import InputStatusArray

rcl_subscription_t subscriber;
//std_msgs__msg__Int32 msg;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#include <Adafruit_NeoPixel.h>
#include "PhysicalButton.h"
#ifdef __AVR__
#include <avr/power.h>
#endif
#define LEDSTRIP_PIN        15
#define NUMPIXELS 18
#define DELAYVAL 25

Adafruit_NeoPixel pixels(NUMPIXELS, LEDSTRIP_PIN, NEO_GRB + NEO_KHZ800);
PhysicalButton button_0(0, 9, "Play", pixels);
//PhysicalButton button_1(9, 9, "Stop", pixels);
//PhysicalButton button_2(18, 9, "X", pixels);
//PhysicalButton button_3(27, 9, "Y", pixels);

//PhysicalButton buttons[] = {button_0, button_1, button_2, button_3};


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(20);
  }
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  //digitalWrite(LED_PIN, (msg->data[0] == 0) ? LOW : HIGH);

  //float percent =  (msg->data.data[0]);
  //int idx = (int) (msg->data.data[1]);
  //pixels.clear();
  //buttons[idx].setPercent(percent );
  float percent = 0.5;
  //button_0.setPercent(percent );

}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_outputs", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/simple_output"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


  //pixels.begin();
  //button_1.setOff();
}

void loop() {
  delay(200);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
