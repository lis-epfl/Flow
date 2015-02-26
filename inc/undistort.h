#ifndef UNDISTORT_H_
#define UNDISTORT_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "settings.h"
#include "utils.h"

#define LENGTH_IPOLY 11
const float IPOLY[]= {10,192.937564,97.98972,1.160704,24.567268,8.627974,-0.85221,-32.956107,-55.815696,-32.128902,-6.209734};

#define LENGTH_POLY 6
const float POLY[]={5,-1.321939e+02,0.000000e+00,6.467560e-04,1.404318e-05,5.239326e-09};

#define CENTER_ROW 106.623265f
#define CENTER_COL 176.347583f

#define AFF_C 1.000372f
#define AFF_D -0.000303f
#define AFF_E -0.000467f

#define HEIGHT_C 240
#define WIDTH_C 376
#define sf 1

uint8_t world2cam(float point2D[2], float point3D[3]);
uint8_t create_maps(float *mapx, float *mapy);
uint8_t undistort_uv(float *mapx,float *mapy,uint8_t *x, uint8_t *y,float *u, float *v);
#endif