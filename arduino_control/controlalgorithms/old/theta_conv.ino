#include <math.h>

// Function to find theta_1
// Arm A anticlock from horizontal
float theta_1(float tar_x, float tar_y, float L_1, float L_2, float counts_per_rotation, float pi) {
  float phi = atan2(tar_y, tar_x);
  float c_sq = pow(tar_x,2) + pow(tar_y,2);
  float alpha_1 = acos((c_sq + pow(L_1,2) - pow(L_2,2))/(2*L_1*sqrt(c_sq)));
  float theta_1_val = ((phi + alpha_1)/(2*pi))*counts_per_rotation;
  return theta_1_val;
}

// Function to find theta_2
// Arm B clockwise from horizontal
float theta_2(float tar_x, float tar_y, float L_1, float L_2, float counts_per_rotation, float pi) {
  float phi = atan2(tar_y, tar_x);
  float c_sq = pow(tar_x,2) + pow(tar_y,2);
  float alpha_1 = acos((c_sq + pow(L_1,2) - pow(L_2,2))/(2*L_1*sqrt(c_sq)));
  float alpha_2 = acos((pow(L_1,2) + pow(L_2,2) - c_sq)/(2*L_1*L_2));
  float theta_2_val = ((PI - phi - alpha_1 - alpha_2)/(2*pi))*counts_per_rotation;
  return theta_2_val;
}
