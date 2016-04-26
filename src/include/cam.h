/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file    cam.h
 *
 * \author  MAV'RIC Team
 * \author  Brice Platerrier
 *
 * \brief   Implements functions used for projection of points and optic-flow
 *			field from sphere to plane and conversely.
 *
 * \detail  Detailed description and important information to use this module
 ******************************************************************************/
#ifndef _CAM_H_
#define _CAM_H_

//#include <float.h>
#include <math.h>
#include "maths.h"
#include "quick_trig.h"

#define MAX_POL_LENGTH 8
#define MAX_INVPOL_LENGTH 8
#define SCALING_FLOW_FACTOR 100

/**
 * @brief Parameters of the unified model of the omnidirectional camera
 */
typedef struct
{
  float pol[MAX_POL_LENGTH];    	    ///< the polynomial coefficients: pol[0] + x*pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                	    ///< length of polynomial
  float invpol[MAX_INVPOL_LENGTH]; 	  ///< the coefficients of the inverse polynomial
  int length_invpol;             	    ///< length of inverse polynomial
  float xc;         				          ///< row coordinate of the center
  float yc;         				          ///< column coordinate of the center
  float c;          				          ///< affine parameter
  float d;          				          ///< affine parameter
  float e;          				          ///< affine parameter
  int width;         				          ///< image width
  int height;        				          ///< image height
}cam_model;

/**
 * @brief Model of the camera
 */
extern const cam_model px4_model;

/**
 * @brief Back-projects a 2D point, in pixel coordinates, onto the unit sphere
 *
 *	@param [xp, yp, zp]				coordinates of the backprojected point (not normalized)
 *  @param [u, v]					    the pixel coordinates of the point
 *  @param *mycam_model 			the model of the calibrated camera
 *
 * @return	cartesian coordinates of the backprojected point (not normalized)
 */
void cam2world(float *xp, float *yp, float *zp, const float u, const float v);

/**
 * @brief Back-projects a 2D optic-flow vector, in pixel coordinates, onto the unit sphere
 *
 *	@param [flow_x, flow_y, flow_z]     spherical coordinates of optic-flow vector
 *	@param [xp, yp, zp]				          coordinates of the backprojected point (not normalized)
 *  @param [u, v]					              the pixel coordinates of the point
 * 	@param [flow_u, flow_v]			        the pixel coordinates of the optic-flow vectors
 *	@param *mycam_model 			          the model of the calibrated camera
 *
 * @return	cartesian coordinates of the optic-flow vector projected on the unit sphere
 */
void flow2world(float *flow_x, float *flow_y, float *flow_z, const float xp, const float yp, const float zp, const float flow_u, const float flow_v);
#endif /* _CAM_H_ */
