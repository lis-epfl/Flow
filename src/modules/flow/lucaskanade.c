#include "lucaskanade.h"

void lucas_kanade( uint8_t * data, uint8_t * data_old, int32_t dsx, int32_t dsy, int32_t roi_sx, int32_t roi_sy, int32_t roi_x, int32_t roi_y, float* num_x, float* num_y, float* den )
{
	// compute gradients
	uint32_t dp = roi_x + roi_y * dsx;

	// int pshift = 8;

	// sums
	dat_t sgxgx=0;
	dat_t sgxgy=0;
	dat_t sgygy=0;
	dat_t sgxgt=0;
	dat_t sgygt=0;

	for (int y=0; y<roi_sy; ++y) {
		for (int x=0; x<roi_sx; ++x) {
			//dat_t gx = data[ dp + 1] - data[ dp ]; //  factor 1
			//dat_t gy = data[ dp + dsx] - data[ dp ];  //  factor 1
			
			dat_t gx = (float)(data[ dp + 1])   - (float)(data[ dp - 1]); //  factor 1/2
			dat_t gy = (float)(data[ dp + dsx]) - (float)(data[ dp - dsx]);  //  factor 1/2

			//dat_t gx = 8 * (data[ dp + 1] - data[ dp - 1]) + data[ dp - 2 ] - data[ dp + 2 ]; //  factor 1/12
			//dat_t gy = 8 * (data[ dp + dsx] - data[ dp - dsx]) + data[ dp - 2*dsx ] - data[ dp + 2*dsx ];  //  factor 1/12

			dat_t gt = (float)(data[ dp ]) - (float)(data_old[ dp ]);  // factor 1

			sgxgx += gx*gx; // factor 1/4  (* 36)
			sgxgy += gx*gy; // factor 1/4  (* 36)
			sgygy += gy*gy; // factor 1/4  (* 36)

			sgxgt -= gx*gt; // factor 1/2  (* 6)
			sgygt -= gy*gt; // factor 1/2  (* 6)

			++dp;
		}
		dp += dsx - roi_sx;
	}
	/*
	sgxgx = sgxgx >> pshift;
	sgxgy = sgxgy >> pshift;
	sgygy = sgygy >> pshift;
	sgxgt = sgxgt >> pshift;
	sgygt = sgygt >> pshift;
	*/
	*den   = sgxgx * sgygy - sgxgy * sgxgy; // factor 1/16  (* 36 * 36)
	*num_x = sgxgt * sgygy - sgygt * sgxgy; // factor 1/8  (* 36 * 6)
	*num_y = - sgxgt * sgxgy + sgygt * sgxgx; // factor 1/8  (* 36 * 6)

	*num_x *= 2; // compensate differing factors
	*num_y *= 2;
}
