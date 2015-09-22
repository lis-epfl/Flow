#ifndef _LUCASKANADE_H_
#define _LUCASKANADE_H_

#include <stdint.h>

typedef float dat_t; //

// dsx : dimension of data in x direction
// dsy : dimension of data in y direction
// roi_x, roi_y : position of region of interest
// stores result as fraction num/den
void lucas_kanade( uint8_t * data, uint8_t * data_old, 
		int32_t dsx, int32_t dsy, 
		int32_t roi_sx, int32_t roi_sy, 
		int32_t roi_x, int32_t roi_y,
		float* num_x, float* num_y, float* den );


#endif // _LUCASKANADE_H_