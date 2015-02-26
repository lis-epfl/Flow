#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

//#include "settings.h"
//#include "utils.h"
#include "flow2.h"


/**
 * @brief Given two images and a point, computes the apparent motion vector at that point using Lucas Kanade
 *
 * @param *Im0 pointer to first image
 * @param *Im1 pointer to second image
 * @param x Horizontal coordinate of the point
 * @param y Vertical coordinate of the point
 * @param *u Pointer to the horizontal component of the vector
 * @param *v POinter to the vertical component of the vector 
 */
uint8_t lk2(float *Im0, float *Im1,uint16_t x, uint16_t y, float *u, float *v){
   uint16_t j=0,k=0;
   uint16_t img_width = global_data.param[PARAM_IMAGE_WIDTH];

  float A11 = 0; 
  float A12 = 0; 
  float A22 = 0; 
  float b1  = 0; 
  float b2  = 0;
  int32_t F2F1 = 0; 
  int32_t F4F3 = 0;
  int32_t FCF0 = 0;

   *u=0;
   *v=0;

   for (j = x-(uint16_t)(WSIZE*0.5f); j < x+(uint16_t)(WSIZE*0.5f); j++)
   {
     for (k = y-(uint16_t)(WSIZE*0.5f); k < y+(uint16_t)(WSIZE*0.5f); k++)
     {

      F2F1 = Im0[(k)*img_width+j+1]-Im0[k*img_width+j-1]; //horizontal differential
      F4F3 = Im0[(k+1)*img_width+j]-Im0[(k-1)*img_width+j]; //vertical differential
      FCF0 = Im0[k*img_width+j]-Im1[k*img_width+j]; //time differential
      
      // update summations
      A11 += F2F1 * F2F1;
      A12 += F4F3 * F2F1;
      A22 += F4F3 * F4F3;
      b1  += FCF0 * F2F1;
      b2  += FCF0 * F4F3;
     }
   }

  float detA = ( (float)(A11)*A22 - (float)(A12)*A12 );

  *u = ( b1 * A22 - b2 * A12 ) / detA;
  *v = ( b2 * A11 - b1 * A12 ) / detA;

   return 0;
}