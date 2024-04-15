#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

rcl_subscription_t subscriber;
//std_msgs__msg__Int32 msg;
//std_msgs__msg__Float32 msg;
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
#define LEDSTRIP_PIN        15
#define NUMPIXELS 18
#define DELAYVAL 25

Adafruit_NeoPixel pixels(NUMPIXELS, LEDSTRIP_PIN, NEO_GRB + NEO_KHZ800);

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void subscription_callback(const void * msgin)
{
  //const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  //const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);

  return;

  /*
  const rosidl_runtime_c__float__Sequence seq = (msg->data);
  if (seq.size == 2) {

    float percent =  (float) (seq.data[1]);
    int idx = (int) (seq.data[1]);
    //int idx = 0;
    pixels.clear();
    //buttons[idx].setPercent(percent );
    setPercent(idx, percent );
    //Serial.print("Working?");  // prints a label


  }
*/
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
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}




//TODO: Button on functions
void setOn(int button_no)
{
  int brightness = 150;
  int color[3] = {brightness, brightness, brightness};
  setColor(button_no, color);
}

void setColor(int button_no, int color[3])
{
  int _start_led_idx = 0 + 9 * button_no;
  int _led_no = 9;
  for (int i = _start_led_idx; i < _led_no + _start_led_idx; i++) {
    pixels.setPixelColor(i, pixels.Color(color[0], color[1], color[2]));
  }
  pixels.show();
}

void setOff(int button_no)
{
  int color[3] = {0, 0, 0};
  setColor(button_no, color);
}

void setHover(int button_no)
{
  int color[3] = {50, 50, 70};
  setColor(button_no, color);
}

void setPercent(int button_no, float percent)
{
  int order_map[9] = {4, 1, 2, 3, 8, 7, 6, 5, 0};

  int color[3] = {0, 150, 150};

  int _start_led_idx = 0 + 9 * button_no;
  int _led_no = 9;

  // Start center and go around
  if (percent < 0.1f) {
    //Starting
    setOff(button_no);
  }
  else if (percent >= 0.1f && percent < 0.9f) {
    // Only center
    //setOff();
    float limit = percent * 100.0 / 9;
    for (int i = 0; i < 9; i++) {
      if (limit > i) {
        //Light the pixel
        //If the difference is small light partially
        float diff = (limit - i) * (limit - i);
        if (diff < 1) {
          pixels.setPixelColor(order_map[i] + _start_led_idx, pixels.Color(color[0] * (diff ), color[1] * (diff), color[2]) * (diff));
        }
        else {
          pixels.setPixelColor(order_map[i] + _start_led_idx, pixels.Color(color[0], color[1], color[2]));
        }
      }
      else {
        pixels.setPixelColor(order_map[i] + _start_led_idx, pixels.Color(0, 0, 0));
      }
    }
  }
  else if (percent >= 0.9f) {
    setOn(button_no);
  }
  pixels.show();

}
