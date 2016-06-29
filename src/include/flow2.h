/*******************************************************************************
 * \file    flow2.h
 *
 * \author  Brice Platerrier
 *
 * \brief   Implements functions for flow processing
 *
 * \detail  Detailed description and important information to use this module
 ******************************************************************************/
#ifndef _FLOW2_H_
#define _FLOW2_H_

#include <math.h>
#include <float.h>
#include "maths.h"
#include "quick_trig.h"

#define COARSE_BINS	8 		// number of coarse bins
#define COARSE_PREC	0.76605f 	// coarse precision (sin(31 degrees/2))
#define REFINED_BINS	7 		// number of refined bins
#define NB_ITERATIONS 	4		// number of iterations in refining the vote
#define REFINED_PREC_1	0.27312f	// refined precision (sin(4.4 degrees/2))
#define REFINED_PREC_2	0.09901f	// refined precision (sin(4.4 degrees/2))
#define REFINED_PREC_3	0.02967f	// refined precision (sin(4.4 degrees/2))
#define REFINED_PREC_4	0.01396f	// refined precision (sin(4.4 degrees/2))

/**
 * @brief Voting bins structure
 */
typedef struct voting_bins{
	float *x;
	float *y;
	float *z;
	float prec;
	uint8_t size;
	uint8_t *acc;
}voting_bins;

/**
 * @brief Refined voting directions structure
 */
typedef struct r_dir_3d{
	float x[REFINED_BINS];
	float y[REFINED_BINS];
	float z[REFINED_BINS];
}r_dir_3d;

/**
* @brief Normalizes vector
*
*   @param [*x, *y, *z]    	cartesian coordinates of the optic flow vector
*
* @return Normalized vector
*/
static inline void normalize(float *x, float *y, float *z)
{
	float invnorm = maths_fast_inv_sqrt(SQR((*x)) + SQR((*y)) + SQR((*z)));
	*x *= invnorm;
	*y *= invnorm;
	*z *= invnorm;
}

/**
* @brief Converts axis-angle representation to rotation matrix
*
*	@param R 					rotation matrix
*   	@param [u_x, u_y, u_z]    			cartesian coordinates of the axis
*	@param c 					cosinus of the angle
*	@param s 					sinus of the angle
*
* @return Rotation matrix coefficients
*/
static inline void aa2mat(float R[9], float u_x, float u_y, float u_z, float c, float s, float t)
{
	R[0] = SQR(u_x)*t + c;
	R[1] = u_x*u_y*t - u_z*s;
	R[2] = u_x*u_z*t + u_y*s;
	R[3] = u_x*u_y*t + u_z*s;
	R[4] = SQR(u_y)*t + c;
	R[5] = u_y*u_z*t - u_x*s;
	R[6] = u_x*u_z*t - u_y*s;
	R[7] = u_y*u_z*t + u_x*s;
	R[8] = SQR(u_z)*t + c;
}

/**
* @brief Proceeds to derotation of the optic-flow vector
*
*   @param [flow_x, flow_y, flow_z]     cartesian coordinates of the optic-flow vector
*   @param [x, y, z]                    cartesian coordinates of the viewing direction
*   @param [x_rate, y_rate, z_rate]     gyroscope rate angles
*
* @return substitutes the initial flow with derotated one
*/
static inline void derotate_flow(float *flow_x, float *flow_y, float *flow_z, float d_x, float d_y, float d_z, float x_rate, float y_rate, float z_rate)
{	
	// normalize direction
	normalize(&d_x, &d_y, &d_z);

	// remove rotational component from optical flow
	*flow_x += (y_rate*d_z)-(z_rate*d_y);
	*flow_y += (z_rate*d_x)-(x_rate*d_z);
	*flow_z += (x_rate*d_y)-(y_rate*d_x);
}

/**
* @brief Computes great circle normal vector
*
*   @param [flow_x, flow_y, flow_z]     spherical coordinates of the optic-flow vector
*   @param [d_x, d_y, d_z]              cartesian coordinates of the viewing direction
*   @param [n_x, n_y, n_z]     			cartesian coordinates of the normal vector
*
* @return normalized normal vector
*/
static inline void great_circle_vector(float *n_x, float *n_y, float *n_z, float flow_x, float flow_y, float flow_z, float d_x, float d_y, float d_z)
{
	*n_x = d_y * flow_z - d_z * flow_y;
	*n_y = d_z * flow_x - d_x * flow_z;
	*n_z = d_x * flow_y - d_y * flow_x;

	normalize(n_x, n_y, n_z);
}

/**
* @brief Proceeds to coarse voting on the unit sphere
*
*   @param *bins     					pointer to the voting bins structure 
*   @param [n_x, n_y, n_z]     			cartesian coordinates of great circle normal vector
*
* @return incremented accumulator for voting
*/
static inline void voting(voting_bins *bins, float n_x, float n_y, float n_z)
{
	for(uint8_t i = 0; i < bins->size; i++){
		// vote along great circle
    		if(maths_f_abs(n_x*bins->x[i] + n_y*bins->y[i] + n_z*bins->z[i]) < bins->prec){
    			bins->acc[i]++;
    		}
	}
}

/**
* @brief Proceeds to refined voting on the unit sphere
*
*   @param *bins     					pointer to the voting bins structure
*   @param prev_prec     				coarse precision
*   @param [n_x, n_y, n_z]     			cartesian coordinates of great circle normal vector
*   @param [best_x, best_y, best_z]		cartesian coordinates of coarse estimate of direction of motion
*
* @return incremented accumulator for refined voting
*/
static inline void refined_voting(voting_bins *bins, float prev_prec, float n_x, float n_y, float n_z, float best_x, float best_y, float best_z)
{
	// only consider samples that would vote for coarse estimate
	if(maths_f_abs(n_x*best_x + n_y*best_y + n_z*best_z) < prev_prec){
		voting(bins, n_x, n_y, n_z);
	}
}

/**
* @brief Finds a coarse estimate of the direction of motion
*
*   @param *bins     					pointer to the voting bins structure
*   @param [best_x, best_y, best_z]     cartesian coordinates of the direction of motion estimate
*
* @return cartesian coordinates of coarse estimate (unnormalized)
*/
void find_best(voting_bins *bins, float *best_x, float *best_y, float *best_z);

/**
* @brief Rotates (refined) voting bins
*
*	@param *bins 						pointer to the voting bins structure
*   @param [best_x, best_y, best_z]    	cartesian coordinates of the coarse estimate
*	@param [r_dir_x, r_dir_y, r_dir_z]	pointers to coordinate arrays
*
* @return Rotated voting bins
*/
void rotate_bins(voting_bins *bins, float best_x, float best_y, float best_z, float *r_dir_x, float *r_dir_y, float *r_dir_z);


/**
* @brief calculates sector wise statistics (max, max_pos, min, min_pos, stddev, avg)
*
*	@param pixel_count 					total number of pixels (sampling points)
*   @param sector_count    				number of sectors
*	@param theta_start 					angle at beginning of field of view
*	@param theta_end 					angle at end of field of view
*	@param flow_x 						flow in x direction
*	@param flow_y 						flow in y direction
*	@param flow_z 						flow in z direction
*	@param flow_z 						angle at each sampling point
*	@param maxima 						address to write maximal values to [millirad/frame]
*	@param max_pos 						address to write position of maximum to [centirad]
*	@param minima 						address to write minimal values to [millirad/frame]
*	@param min_pos 						address to write position of minimum to [centirad]
*	@param stddev 						address to write standard deviation to values to [millirad/frame]
*	@param avg 	 						address to write average values to [millirad/frame]
*
* @return Rotated voting bins
*/
static inline void calc_flow_stats(uint16_t pixel_count,
									uint8_t sector_count,
									float theta_start,
									float theta_end,
									float flow_x[],
									float flow_y[],
									float flow_z[],
									float theta[],
									uint16_t maxima[],
									uint8_t max_pos[],
									int16_t minima[],
									uint8_t min_pos[],
									int16_t stddev[],
									int16_t avg[])
{
	// iterate over all sectors
	uint16_t i_pix = 0;
	float dTheta = (theta_end-theta_start)/sector_count;
	float theta0 = theta_start;												// angle at beginning of the sector
	float theta1 = theta0 + dTheta;											// angle at end of the sector
	for(uint8_t i_sec = 0; i_pix < pixel_count && i_sec < sector_count; i_sec++)
	{
		float maximum =  FLT_MIN;
		float minimum =  FLT_MAX;
		float sum = 0;
		float sum2 = 0;
		uint8_t max_ind = 0;
		uint8_t min_ind = 0;
		uint16_t sec_start_ind = i_pix;
		while(i_pix < pixel_count && theta[i_pix] < theta1)
		{
			float flow2 = SQR(*(flow_x++)) + SQR(*(flow_y++)) + SQR(*(flow_z++));	// norm of flow squared
			float flow = maths_fast_sqrt(flow2);									// norm of flow
			sum += flow;															// sum of flow for sector
			sum2 += flow2;															// sum of flow squared for sector

			if(flow > maximum){
				maximum = flow;
				max_ind = i_pix;
			}else if(flow < minimum)
			{
				minimum = flow;
				min_ind = i_pix;
			}
			i_pix++;
		}
		uint16_t sector_size = i_pix - sec_start_ind;
		if(sector_size > 0)
		{
			*(maxima++) = (uint16_t)(1000*maximum);										// [millirad]
			*(minima++) = (uint16_t)(1000*minimum);										// [millirad]
			*(stddev++) = (uint16_t)(1000*maths_fast_sqrt((sum2 - SQR(sum))/sector_size)); // standard deviation of flow of sector [millirad]
			*(avg++) = (uint16_t)(1000*sum/sector_size);								// average of flow of sector [millirad]
			*(min_pos++) = (uint8_t)(100*(theta[min_ind]-theta0));	// transform index to relative azimuth [centirad]
			*(max_pos++) = (uint8_t)(100*(theta[max_ind]-theta0));	// transform index to relative azimuth [centirad]
		}else
		{
			*(maxima++) = 0;
			*(minima++) = 0;
			*(stddev++) = 0;
			*(avg++) = 0;
			*(min_pos++) = 0;
			*(max_pos++) = 0;
		}	

		theta0 += dTheta;
		theta1 += dTheta;
	}
}

#endif /* _FLOW2_H_ */
