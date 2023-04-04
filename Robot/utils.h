#ifndef UTILS_H
#define UTILS_H

#define PI 3.1415926535897932384626433832795028841971693993751058209

#include <math.h>
#include <stdlib.h>

float remap(float v, float a1, float b1, float a2, float b2);
float remap(int v, float a1, float b1, float a2, float b2);
int remap(float v, int a1, int b1, int a2, int b2);
int remap(int v, int a1, int b1, int a2, int b2);

class Timer{
public:
  Timer(unsigned long delta);
  Timer(unsigned long delta, unsigned long time);

  void reset(unsigned long time);
  bool check(unsigned long time);

private:
  unsigned long time;
  unsigned long delta;
};

#endif  // UTILS_H