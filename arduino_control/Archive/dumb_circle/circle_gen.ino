#include <math.h>

float tarx(float o_x, float r, float th0){
  return o_x + (r * cos(th0));
}

float tary(float o_y, float r, float th0){
  return o_y + (r * sin(th0));
}
