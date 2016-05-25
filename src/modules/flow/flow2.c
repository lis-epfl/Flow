/*******************************************************************************
 * \file    flow2.c
 *
 * \author  Brice Platerrier
 *
 * \brief   Implements functions for flow processing
 *
 * \detail  Detailed description and important information to use this module
 ******************************************************************************/
#include "flow2.h"

void find_best(voting_bins *bins, float *best_x, float *best_y, float *best_z)
{
	uint8_t best = 0;	// highest accumulated value
	uint8_t best_n = 0;	// number of indices with same accumulated value
	for(uint8_t i = 0; i < bins->size; i++){
		if(bins->acc[i] > best){
			best = bins->acc[i];
			best_n = 1;
		} 
		else if(bins->acc[i] == best){
			best_n++;
		}
	}
	*best_x = 0;
	*best_y = 0;
	*best_z = 0;
	uint8_t cnt = 0;
	for(uint8_t j = 0; j < bins->size; j++){
		if(bins->acc[j]==best){
			// *best_x += bins->x[j]/best_n;
			// *best_y += bins->y[j]/best_n;
			// *best_z += bins->z[j]/best_n;
			*best_x = bins->x[j];
			*best_y = bins->y[j];
			*best_z = bins->z[j];
			cnt++;
		}
		if(cnt == best_n){
			break;
		}
	}
	if(cnt > 1){
		normalize(best_x, best_y, best_z);
	}
}

void rotate_bins(voting_bins *bins, float best_x, float best_y, float best_z, float *r_dir_x, float *r_dir_y, float *r_dir_z)
{
	if(best_x==0.0f, best_y==0.0f, best_z==-1.0f){
		for(uint8_t i=0; i<bins->size; i++){
			bins->x[i] = r_dir_x[i];
			bins->y[i] = r_dir_y[i];
			bins->z[i] = r_dir_z[i];
		} 
	}
	else{
		// compute rotation matrix (saves time)
		float c = -best_z;
		float s = maths_fast_sqrt(1 - SQR(c));
		float u_x = 1.0f * best_y; //0f * best_z - (-1f) * best_y
		float u_y = -1.0f * best_x; //-1f * best_x - 0f * best_z
		float u_z = 0.0f; //0f * best_y - 0f * best_x
		float R[9];
		aa2mat(R, u_x, u_y, u_z, c, s);

		// compute rotated refined bins
		for(uint8_t i=0; i<bins->size; i++){
			bins->x[i] = r_dir_x[i]*R[0] + r_dir_y[i]*R[1] + r_dir_z[i]*R[2];
			bins->y[i] = r_dir_x[i]*R[3] + r_dir_y[i]*R[4] + r_dir_z[i]*R[5];
			bins->z[i] = r_dir_x[i]*R[6] + r_dir_y[i]*R[7] + r_dir_z[i]*R[8];
		} 
	}
}