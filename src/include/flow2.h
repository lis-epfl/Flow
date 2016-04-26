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

#define COARSE_BINS	42 // number of coarse bins
#define REFINED_BINS	37 // number of refined bins

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

#endif /* _FLOW2_H_ */
