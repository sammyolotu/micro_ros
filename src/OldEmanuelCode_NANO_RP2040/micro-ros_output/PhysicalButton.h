#ifndef PhysicalButton_h
#define PhysicalButton_h

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>


class PhysicalButton
{
  public:
    PhysicalButton(int start_led_idx, int led_no, String id, Adafruit_NeoPixel pixels);
    void setOn();
    void setOn(int[3]);
    void setColor(int[3]);
    void setHover();
    void setOff();
    void setPercent(float percent);

  private:
    int _start_led_idx;
    int _led_no;
    String _id;
    Adafruit_NeoPixel _pixels;
    //int _color[];
    //Internally have a 3x3 array?
    //Array for color?

};


PhysicalButton::PhysicalButton(int start_led_idx, int led_no,  String id, Adafruit_NeoPixel pixels)
{
  _start_led_idx = start_led_idx;
  _led_no = led_no; //9?
  _id = id;
  _pixels = pixels;
  //_color to black?

}

//TODO: Button on functions
void PhysicalButton::setOn()
{
  int brightness = 150;
  int color[3] = {brightness, brightness, brightness};
  setColor(color);
}

void PhysicalButton::setColor(int color[3])
{
  for (int i = _start_led_idx; i < _led_no+_start_led_idx; i++) {
    _pixels.setPixelColor(i, _pixels.Color(color[0], color[1], color[2]));
  }
  _pixels.show();
}

void PhysicalButton::setOff()
{
  int color[3] = {0, 0, 0};
  setColor(color);
}

void PhysicalButton::setHover()
{
  int color[3] = {50, 50, 70};
  setColor(color);
}

void PhysicalButton::setPercent(float percent)
{
  int order_map[9] = {4, 1, 2, 3, 8, 7, 6, 5, 0};

  int color[3] = {0, 150, 150};



  // Start center and go around
  if (percent < 0.1f) {
    //Starting
    setOff();
  }
  else if (percent >= 0.1f && percent < 0.9f) {
    // Only center
    //setOff();
    float limit = percent * 100.0 / 9;
    for (int i = 0; i < 9; i++) {
      if (limit > i) {
        //Light the pixel
        //If the difference is small light partially
        float diff = (limit - i)*(limit - i);
        if (diff < 1) {
          _pixels.setPixelColor(order_map[i] + _start_led_idx, _pixels.Color(color[0]*(diff ), color[1]*(diff), color[2])*(diff));
        }
        else {
          _pixels.setPixelColor(order_map[i] + _start_led_idx, _pixels.Color(color[0], color[1], color[2]));
        }
      }
      else {
        _pixels.setPixelColor(order_map[i] + _start_led_idx, _pixels.Color(0, 0, 0));
      }
    }
  }
  else if (percent >= 0.9f) {
    setOn();
  }
  _pixels.show();

}


#endif
