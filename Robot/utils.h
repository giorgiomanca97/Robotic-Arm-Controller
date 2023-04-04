#ifndef UTILS_H
#define UTILS_H

#define PI 3.1415926535897932384626433832795028841971693993751058209


#include <math.h>


float remap(float v, float a1, float b1, float a2, float b2, bool clamp = false);
float remap( long v,  long a1,  long b1, float a2, float b2, bool clamp = false);

 long remap(float v, float a1, float b1,  long a2,  long b2, bool clamp = false);
 long remap( long v,  long a1,  long b1,  long a2,  long b2, bool clamp = false);


class Timer{
public:
  Timer();
  Timer(unsigned long delta);
  Timer(unsigned long delta, unsigned long time);

  void setup(unsigned long delta);
  
  void reset(unsigned long time);
  bool check(unsigned long time);

private:
  unsigned long time;
  unsigned long delta;
};


#endif  // UTILS_H