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
#include "maths.h"
#include "quick_trig.h"

#define COARSE_BINS	42 		// number of coarse bins
#define REFINED_BINS	37 		// number of refined bins
#define COARSE_PREC	0.17364f 	// coarse precision (sin(20 degrees/2))
#define REFINED_PREC	0.03489f	// refined precision (sin(4 degrees/2))

/**
 * @brief Directions of coarse bins
 */
typedef struct {
	float x[COARSE_BINS];
	float y[COARSE_BINS];
	float z[COARSE_BINS];
} coarse_bins;

/**
 * @brief Directions of refined bins around [0 0 1]
 */
typedef struct {
	float x[REFINED_BINS];
	float y[REFINED_BINS];
	float z[REFINED_BINS];
} refined_bins;

/**
 * @brief Coarse bins structure
 */
extern const coarse_bins CB;

/**
 * @brief Refined bins structure
 */
extern const refined_bins RB;

/**
* @brief Proceeds to derotation of the optic-flow vector
*
*   @param [flow_x, flow_y, flow_z]     spherical coordinates of the optic-flow vector
*   @param [x, y, z]                    cartesian coordinates of the viewing direction
*   @param [x_rate, y_rate, z_rate]     gyroscope rate angles
*
* @return substitutes the initial flow with derotated one
*/
void derotate_flow(float *flow_x, float *flow_y, float *flow_z, float d_x, float d_y, float d_z, const float x_rate, const float y_rate, const float z_rate);

/**
* @brief Proceeds to coarse voting on the unit sphere
*
*   @param *acc     			pointer to the voting accumulator (MUST be zeros)
*   @param [flow_x, flow_y, flow_z]     cartesian coordinates of the optic flow vector
*   @param [d_x, d_y, d_z]     		cartesian coordinates of viewing direction corresponding to the optic flow vector
*
* @return incremented accumulator for coarse voting
*/
void coarse_voting(uint8_t *acc, float flow_x, float flow_y, float flow_z, float d_x, float d_y, float d_z);

/**
* @brief Proceeds to refined voting on the unit sphere
*
*   @param *acc     			pointer to the voting accumulator (MUST be zeros)
*   @param [flow_x, flow_y, flow_z]     cartesian coordinates of the optic flow vector
*   @param [d_x, d_y, d_z]     		cartesian coordinates of viewing direction corresponding to the optic flow vector
*   @param [best_x, best_y, best_z]	cartesian coordinates of coarse estimate of direction of motion
*
* @return incremented accumulator for refined voting
*/
void refined_voting(uint8_t *acc, float flow_x, float flow_y, float flow_z, float d_x, float d_y, float d_z, float best_x, float best_y, float best_z);

/**
* @brief Finds a coarse estimate of the direction of motion
*
*   @param *acc     			pointer to the voting accumulator
*   @param [best_x, best_y, best_z]     cartesian coordinates of the optic flow vector
*
* @return cartesian coordinates of coarse estimate
*/
void find_coarse_best(uint8_t *acc, float *best_x, float *best_y, float *best_z);

/**
* @brief Finds a refined estimate of the direction of motion [UNFINISHED: ADD ROTATION OF VOTING BINS]
*
*   @param *acc     			pointer to the voting accumulator
*   @param [best_x, best_y, best_z]    	cartesian coordinates of the optic flow vector
*
* @return cartesian coordinates of refined estimate
*/
void find_refined_best(uint8_t *acc, float *best_x, float *best_y, float *best_z);

#endif /* _FLOW2_H_ */
