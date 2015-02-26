#ifndef UNDISTORT_H_
#define UNDISTORT_H_

#include <stdlib.h>

#include <stdint.h>

//#define LENGTH_IPOLY 11
//const float IPOLY[]= {10,192.937564,97.98972,1.160704,24.567268,8.627974,-0.85221,-32.956107,-55.815696,-32.128902,-6.209734};

const float POLY[]={5,-1.321939e+02,0.000000e+00,6.467560e-04,1.404318e-05,5.239326e-09};

#define HEIGHT_C 240
#define WIDTH_C 376

void cam2world(float point2D[2],float point3D[3]);
uint8_t derotate_sphere(float point3D0[3],float point3D1[3],float motion[3],float wx,float wy,  float wz);
uint8_t point3d_to_angles(float point3D[3], float angles[2]);
#endif