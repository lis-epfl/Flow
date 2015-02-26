#ifndef FLOW2_H_
#define FLOW2_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "settings.h"
#include "utils.h"
#define WSIZE 5
#define N 2
#define M 25
#define f 1
/**
 * @brief Computes pixel flow from image1 to image2
 */
uint8_t lk(float *Im0, float *Im1,uint16_t x, uint16_t y, float *u, float *v);
uint8_t derotate_uv(float *u, float *v,float x, float y, float wx, float wy, float wz);

#endif /* FLOW_H_ */