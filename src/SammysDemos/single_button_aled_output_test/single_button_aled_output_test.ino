
// microros files
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joy.h>

// Arduino imports
#include <WiFiNINA.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

// Neopixels
#include <Adafruit_NeoPixel.h>

#define LED_PIN 4  // Define the pin where the data line is connected, which is a GPIO pin

#define NUM_LEDS 21  //9  // Number of LEDs in the strip

#define NUM_BUTTONS 21  // Number of LEDs in the strip

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


static sensor_msgs__msg__Joy joy_msg;

rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;




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



void subscription_callback(const void *msgin) {
  const sensor_msgs__msg__Joy *msg = (const sensor_msgs__msg__Joy *)msgin;
  //Serial.print("Message received");

  // Set all pixel colors to 'off'
  pixels.clear();

  /*
  // Wait for the A button to be pressed
  if (msg->buttons.size > 0) {
    //&& msg->buttons.data[0] == 1
    // TODO: Do minimum of size and led
    for (int i = 0; i < msg->buttons.size; i++) {
      if (msg->buttons.data[i] == 1) {
        pixels.setPixelColor(i, pixels.Color(255, 255, 255));  // Set pixel color to green
      }
    }
    pixels.show();  // Update the pixel with new settings
  }

  */

  if (msg->axes.size > 0) {
    int LEFT_STICK_X = 0; 
    int LEFT_STICK_Y = 1;
    int RIGHT_STICK_X = 4;

    int UP_ARROW_LED = 1;
    int DOWN_ARROW_LED = 7;
    int LEFT_ARROW_LED = 5;
    int RIGHT_ARROW_LED = 3;
    int CLOCKWISE_ARROW_LED = 2;
    int ANTICLOCKWISE_ARROW_LED = 6;
    
    
    float ACTIVATION_THRESHOLD = 0.8;

    if (msg->axes.data[LEFT_STICK_X] < -ACTIVATION_THRESHOLD) {
      float intensity = sqrt(- msg->axes.data[LEFT_STICK_X]);
      pixels.setPixelColor(RIGHT_ARROW_LED, pixels.Color(255*intensity, 255*intensity, 255*intensity));  
    }
    if (msg->axes.data[LEFT_STICK_X] > ACTIVATION_THRESHOLD) {
      float intensity = sqrt(msg->axes.data[LEFT_STICK_X]);
      pixels.setPixelColor(LEFT_ARROW_LED, pixels.Color(255*intensity, 255*intensity, 255*intensity));  
    }
    if (msg->axes.data[LEFT_STICK_Y] > ACTIVATION_THRESHOLD) {
      float intensity =  sqrt(msg->axes.data[LEFT_STICK_Y]);
      pixels.setPixelColor(UP_ARROW_LED, pixels.Color(255*intensity, 255*intensity, 255*intensity));  
    }
    if (msg->axes.data[LEFT_STICK_Y] < -ACTIVATION_THRESHOLD) {
      float intensity = sqrt(- msg->axes.data[LEFT_STICK_Y]);
      pixels.setPixelColor(DOWN_ARROW_LED, pixels.Color(255*intensity, 255*intensity, 255*intensity));  
    }
    if (msg->axes.data[RIGHT_STICK_X] > ACTIVATION_THRESHOLD) {
      float intensity =  sqrt(msg->axes.data[RIGHT_STICK_X]);
      pixels.setPixelColor(CLOCKWISE_ARROW_LED, pixels.Color(255*intensity, 255*intensity, 255*intensity)); 
    }
    if (msg->axes.data[RIGHT_STICK_X] < -ACTIVATION_THRESHOLD) {
      float intensity = sqrt(- msg->axes.data[RIGHT_STICK_X]);
      pixels.setPixelColor(ANTICLOCKWISE_ARROW_LED, pixels.Color(255*intensity, 255*intensity, 255*intensity));  
    } 



    pixels.show();  // Update the pixel with new settings
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

  // Setup neopixels
  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)

  // Serial coms
  Serial.begin(115200);  // Initialize serial at 115200 baud rate
  while (!Serial) {
    ;  // Wait for the serial port to connect - necessary for some devices
  }

  // Microros setup
  set_microros_transports();


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
