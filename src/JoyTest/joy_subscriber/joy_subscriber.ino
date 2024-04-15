#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joy.h>
static sensor_msgs__msg__Joy joy_msg;

rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN LED_BUILTIN

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1000);
  }
}

// Use to change the light based on the topic
// To turn off
// ros2 topic pub /micro_ros_arduino_subscriber std_msgs/msg/Int32 data:\ 1\ 
// To turn on
// ros2 topic pub /micro_ros_arduino_subscriber std_msgs/msg/Int32 data:\ 0\ 

void subscription_callback(const void* msgin) {
  digitalWrite(LED_PIN, LOW);  // Turn on the LED

  
    const sensor_msgs__msg__Joy *msg = (const sensor_msgs__msg__Joy *)msgin;
    digitalWrite(LED_PIN, LOW); // Turn on the LED
    
    // Assuming the A button corresponds to the first button
    if (msg->buttons.size > 0 && msg->buttons.data[0] == 1) {
      digitalWrite(LED_PIN, HIGH); // Turn on the LED
    } else {
      digitalWrite(LED_PIN, LOW); // Turn off the LED
    }
}

void setup() {
  // Initializing the joy_msg
  // TODO
  // https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/#sequence-types-in-micro-ros

  // try both and observe results.
  /*
  // Assigning dynamic memory to the sequence
  joy_msg.values.capacity = 100;
  joy_msg.values.data = (int32_t*) malloc(mymsg.values.capacity * sizeof(int32_t));
  joy_msg.values.size = 0;
  */

  // Assigning static memory
  static char memory[100];
  joy_msg.header.frame_id.capacity = 100;
  joy_msg.header.frame_id.data = memory;
  //(char*) malloc(joy_msg.header.frame_id.capacity * sizeof(char));
  joy_msg.header.frame_id.size = 0;
  joy_msg.header.stamp.nanosec = 0;
  joy_msg.header.stamp.sec = 0;

  static float memory_ax[20];
  joy_msg.axes.capacity = 20;
  joy_msg.axes.data = memory_ax;
  joy_msg.axes.size = 0;

  static int32_t memory_bt[30];
  joy_msg.buttons.capacity = 30;
  joy_msg.buttons.data = memory_bt;
  joy_msg.buttons.size = 0;


  Serial.begin(115200);  // Initialize serial at 115200 baud rate
  while (!Serial) {
    ;  // Wait for the serial port to connect - necessary for some devices
  }

  /*
  // Assign space
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 50;
  conf.max_ros2_type_sequence_capacity = 20;
  conf.max_basic_type_sequence_capacity = 20;

  bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, joy_msg, Joy),
  &mymsg,
  conf );
  
  Serial.print("RCCHECK failed: ");
  */
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_joy_subscriber", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),  // CAN BE THIS LINE
    "joy"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &joy_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(1);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
