/*******************************************************************************
 * \file    flow2.c
 *
 * \author  Brice Platerrier
 *
 * \brief   Implements functions used for projection of points and optic-flow
 *			field from sphere to plane and conversely.
 *
 ******************************************************************************/
#include "flow2.h"

/* coarse voting bins directions on unit sphere */
const coarse_bins CB = {
	{ 0.85065f, 0.85065f, 0.52573f, 0.52573f, 0.00000f, 0.00000f, 0.00000f, 0.00000f,-0.52573f,-0.52573f,-0.85065f,-0.85065f, 1.00000f, 0.80902f, 0.80902f, 0.50000f, 0.50000f, 0.80902f, 0.80902f, 0.50000f, 0.50000f, 0.30902f, 0.30902f, 0.00000f, 0.30902f, 0.30902f, 0.00000f, 0.00000f,-0.30902f,-0.50000f,-0.30902f,-0.50000f, 0.00000f,-0.30902f,-0.50000f,-0.30902f,-0.50000f,-0.80902f,-0.80902f,-0.80902f,-0.80902f,-1.00000f},
	{ 0.52573f, -0.52573f, 0.00000f, 0.00000f, 0.85065f, 0.85065f,-0.85065f,-0.85065f, 0.00000f, 0.00000f, 0.52573f,-0.52573f, 0.00000f, 0.30902f, 0.30902f, 0.80902f, 0.80902f,-0.30902f,-0.30902f,-0.80902f,-0.80902f, 0.50000f,-0.50000f, 0.00000f, 0.50000f,-0.50000f, 0.00000f, 1.00000f, 0.50000f, 0.80902f, 0.50000f, 0.80902f,-1.00000f,-0.50000f,-0.80902f,-0.50000f,-0.80902f, 0.30902f,-0.30902f, 0.30902f,-0.30902f, 0.00000f},
	{ 0.00000f, 0.00000f, 0.85065f,-0.85065f, 0.52573f,-0.52573f, 0.52573f,-0.52573f, 0.85065f,-0.85065f, 0.00000f, 0.00000f, 0.00000f, 0.50000f,-0.50000f, 0.30902f,-0.30902f, 0.50000f,-0.50000f, 0.30902f,-0.30902f, 0.80902f, 0.80902f, 1.00000f,-0.80902f,-0.80902f,-1.00000f, 0.00000f, 0.80902f, 0.30902f,-0.80902f,-0.30902f, 0.00000f, 0.80902f, 0.30902f,-0.80902f,-0.30902f, 0.50000f, 0.50000f,-0.50000f,-0.50000f, 0.00000f}
};

/* refined voting bins directions on unit sphere */
const refined_bins RB = {
	{ 0.25604f, 0.17389f, 0.08795f, 0.00000f,-0.08795f,-0.17389f,-0.25604f, 0.22073f, 0.13455f, 0.04522f,-0.04522f,-0.13455f,-0.22073f, 0.18161f, 0.09195f, 0.00000f,-0.09195f,-0.18161f, 0.13910f, 0.04677f,-0.04677f,-0.13910f, 0.22073f, 0.18161f, 0.13910f, 0.13455f, 0.09195f, 0.04677f, 0.04522f, 0.00000f,-0.04677f,-0.04522f,-0.09195f,-0.13910f,-0.13455f,-0.18161f,-0.22073f},
	{ 0.00000f, 0.00000f, 0.00000f, 0.00000f, 0.00000f, 0.00000f, 0.00000f, 0.07143f, 0.07257f, 0.07316f, 0.07316f, 0.07257f, 0.07143f, 0.14692f, 0.14877f, 0.14941f, 0.14877f, 0.14692f, 0.22507f, 0.22703f, 0.22703f, 0.22507f,-0.07143f,-0.14692f,-0.22507f,-0.07257f,-0.14877f,-0.22703f,-0.07316f,-0.14941f,-0.22703f,-0.07316f,-0.14877f,-0.22507f,-0.07257f,-0.14692f,-0.07143f},
	{ 0.96667f, 0.98476f, 0.99613f, 1.00000f, 0.99613f, 0.98476f, 0.96667f, 0.97272f, 0.98825f, 0.99629f, 0.99629f, 0.98825f, 0.97272f, 0.97233f, 0.98459f, 0.98878f, 0.98459f, 0.97233f, 0.96436f, 0.97276f, 0.97276f, 0.96436f, 0.97272f, 0.97233f, 0.96436f, 0.98825f, 0.98459f, 0.97276f, 0.99629f, 0.98878f, 0.97276f, 0.99629f, 0.98459f, 0.96436f, 0.98825f, 0.97233f, 0.97272f}
};

void derotate_flow(float *flow_x, float *flow_y, float *flow_z, float d_x, float d_y, float d_z, const float x_rate, const float y_rate, const float z_rate)
{	
	// normalize direction
	float invnorm = maths_fast_inv_sqrt(SQR(d_x) + SQR(d_y) + SQR(d_z));
	d_x *= invnorm;
	d_y *= invnorm;
	d_z *= invnorm;

	// remove rotational component from optical flow
	*flow_x += (y_rate*d_z)-(z_rate*d_y);
	*flow_y += (z_rate*d_x)-(x_rate*d_z);
	*flow_z += (x_rate*d_y)-(y_rate*d_x);
}

void coarse_voting(uint8_t *acc, float flow_x, float flow_y, float flow_z, float d_x, float d_y, float d_z){
	float n_x = d_y * flow_z - d_z * flow_y;
	float n_y = d_z * flow_x - d_x * flow_z;
	float n_z = d_x * flow_y - d_y * flow_x;

	// normalize n
	float invnorm = maths_fast_inv_sqrt(SQR(n_x) + SQR(n_y) + SQR(n_z));
	n_x *= invnorm;
	n_y *= invnorm;
	n_z *= invnorm;

	for(uint8_t i = 0; i < COARSE_BINS; i++){
		// vote along great circle in the correct hemisphere (disambiguate possible positions using angle between flow and bin)
    	if((maths_f_abs(n_x*CB.x[i] + n_y*CB.y[i] + n_z*CB.z[i]) < COARSE_PRECISION) && (flow_x*CB.x[i] + flow_y*CB.y[i] + flow_z*CB.z[i] < 0.0f)){
    		*acc[i] += 1;
    	}
	}
}

void refined_voting(uint8_t *acc, float flow_x, float flow_y, float flow_z, float d_x, float d_y, float d_z, float best_x, float best_y, float best_z){
	float n_x = d_y * flow_z - d_z * flow_y;
	float n_y = d_z * flow_x - d_x * flow_z;
	float n_z = d_x * flow_y - d_y * flow_x;

	// normalize n
	float invnorm = maths_fast_inv_sqrt(SQR(n_x) + SQR(n_y) + SQR(n_z));
	n_x *= invnorm;
	n_y *= invnorm;
	n_z *= invnorm;

	// first test if the great circle would vote in the coarse region
	if(n_x*best_x + n_y*best_y + n_z*best_z) < COARSE_PRECISION){
		for(uint8_t i = 0; i < REFINED_BINS; i++){
			// vote along great circle in the coarse region
    			if(maths_f_abs(n_x*RB.x[i] + n_y*RB.y[i] + n_z*RB.z[i]) < REFINED_PRECISION){
    				*acc[i]++;
    			}
		}
	}
}

void find_coarse_best(uint8_t *acc, float *best_x, float *best_y, float *best_z){
	uint8_t best = 0;	// highest accumulated value
	uint8_t best_n = 0;	// number of indices with same accumulated value
	for(uint8_t i = 0; i < COARSE_BINS; i++){
		if(*acc[i] > best){
			best = *acc[i];
			*best_x = CB.x[i];
			*best_y = CB.y[i];
			*best_z = CB.z[i];
			best_n = 1;
		}
		// if identical accumulated values, choose the mean as the best direction 
		else if(*acc[i] == best){
			*best_x = ((*best_x)*best_n + CB.x[i])/(best_n + 1);
			*best_y = ((*best_y)*best_n + CB.y[i])/(best_n + 1);
			*best_z = ((*best_z)*best_n + CB.z[i])/(best_n + 1);
			best_n++;
		}
	}
	// normalize best for refined voting
	float invnorm = maths_fast_inv_sqrt(SQR((*best_x)) + SQR((*best_y)) + SQR((*best_z)));
	*best_x *= invnorm;
	*best_y *= invnorm;
	*best_z *= invnorm;
}

void find_refined_best(uint8_t *acc, float *best_x, float *best_y, float *best_z){
	uint8_t best = 0;	// highest accumulated value
	uint8_t best_n = 0;	// number of indices with same accumulated value
	for(uint8_t i = 0; i < REFINED_BINS; i++){
		if(*acc[i] > best){
			best = *acc[i];
			*best_x = RB.x[i];
			*best_y = RB.y[i];
			*best_z = RB.z[i];
			best_n = 1;
		}
		// if identical accumulated values, average over best directions 
		else if(*acc[i] == best){
			*best_x = ((*best_x)*best_n + RB.x[i])/(best_n + 1);
			*best_y = ((*best_y)*best_n + RB.y[i])/(best_n + 1);
			*best_z = ((*best_z)*best_n + RB.z[i])/(best_n + 1);
			best_n++;
		}
	}
}

