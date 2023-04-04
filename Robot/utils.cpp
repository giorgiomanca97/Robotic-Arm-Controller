#include "utils.h"

float remap(float v, float a1, float b1, float a2, float b2) {
  return a2 + (v - a1) / (b1 - a1) * (b2 - a2);
}

float remap(int v, float a1, float b1, float a2, float b2) {
  return remap((float)v, a1, b1, a2, b2);
}

int remap(float v, int a1, int b1, int a2, int b2) {
  return (int)remap(v, (float)a1, (float)b1, (float)a2, (float)b2);
}

int remap(int v, int a1, int b1, int a2, int b2) {
  return (int)remap((float)v, (float)a1, (float)b1, (float)a2, (float)b2);
}


Timer::Timer(unsigned long delta){
  this->delta = delta;
  this->time = 0;
}

Timer::Timer(unsigned long delta, unsigned long time){
  this->delta = delta;
  this->time = time;
}

void Timer::reset(unsigned long time){
  this->time = time;
}

bool Timer::check(unsigned long time){
  unsigned long dt;

  if(time < this->time){
    dt = 4294967295 - this->time;
    dt = time + dt + 1;
  } else {
    dt = time - this->time;
  }

  if(dt >= delta){
    this->time = time;
    return true;
  } else {
    return false;
  }
}
