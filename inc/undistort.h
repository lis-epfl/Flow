#ifndef UNDISTORT_H_
#define UNDISTORT_H_

#include <stdlib.h>

#include <stdint.h>

//TODO: make this value configurable
const float POLY[]={5,-1.321939e+02,0.000000e+00,6.467560e-04,1.404318e-05,5.239326e-09};

#define HEIGHT_C 240
#define WIDTH_C 376

void cam2world(float point2D[2],float point3D[3]);
uint8_t derotate_sphere(float point3D0[3],float point3D1[3],float motion[3],float wx,float wy,  float wz);
uint8_t point3d_to_angles(float point3D[3], float angles[2]);
#endif