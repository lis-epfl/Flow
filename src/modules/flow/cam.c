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
 * \file cam.c
 *
 * \author MAV'RIC Team
 * \author Brice Platerrier
 *
 * \brief Useful functions related to the camera model
 *
 ******************************************************************************/

#include "cam.h"

const cam_model px4_model= {{6.660506e+01f, 0.0f, -6.426152e-03f, 2.306550e-05f, -2.726345e-07f}, // signs were changed
			5, 
			{98.889649f, 60.099030f, 3.523247f, 11.584154f, 10.704617f, 4.911849f, 0.899849f},
			7,
			56.232012f,
			77.63939272f,
			1.001183f,
			0.001337f,
			0.002268f,
			160,
			120
			};

void cam2world(float *xp, float *yp, float *zp, const float u, const float v)
{
	 float *pol    = &(px4_model.pol);
	 float xc      = (px4_model.xc);
	 float yc      = (px4_model.yc); 
	 float c       = (px4_model.c);
	 float d       = (px4_model.d);
	 float e       = (px4_model.e);
	 uint8_t length_pol = (px4_model.length_pol); 
	 float invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

	 // back-projection of u and v
	 *xp = invdet*(    (u - xc) - d*(v - yc) );
	 *yp = invdet*( -e*(u - xc) + c*(v - yc) );
	  
	 float r   = maths_fast_sqrt(SQR((*xp)) + SQR((*yp))); //distance [pixels] of  the point from the image center
	 *zp  = pol[0];
	 
	 float r_i = 1;
	 
	 // compute z from polynomial model
	 for (uint8_t i = 1; i < length_pol; i++)
	 {
	   r_i *= r;
	   *zp  += r_i*pol[i];
	 }
}

void flow2world(float *flow_x, float *flow_y, float *flow_z, const float xp, const float yp, const float zp, const float flow_u, const float flow_v)
{
	 float *pol    = &(px4_model.pol);
	 float c       = (px4_model.c);
	 float d       = (px4_model.d);
	 float e       = (px4_model.e);
	 uint8_t length_pol = (px4_model.length_pol); 
	 float invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file
	 
	 
	 // transformation from rectangular to spherical coordinates
	 float r   = maths_fast_sqrt(SQR(xp) + SQR(yp));
	 float theta = quick_trig_atan2(yp,xp); // could be replaced by helper function
	 float phi = quick_trig_atan2(r, zp); 
	 
	 float r_i = 1;
	 float dzp = pol[1];
	 
	 // compute polynomial model derivative
	 for (uint8_t i=2; i < length_pol; i++)
	 {
	   r_i *=r;
	   dzp += i*pol[i]*r_i;
	 }
	 
	 // project optic-flow on unit sphere (in spherical coordinates)
	 float d_thetau = invdet*(-yp/SQR(r) - e*xp/SQR(r));
	 float d_thetav = invdet*(d*yp/SQR(r) + c*xp/SQR(r));
	 float d_phir = (zp - r*dzp)/(SQR(zp) + SQR(r));
	 float d_phiu = d_phir*invdet*(xp/r - e*yp/r);
	 float d_phiv = d_phir*invdet*(-d*xp/r + c*yp/r);
	 
	 float flow_theta = d_thetau*flow_u + d_thetav*flow_v;
	 float flow_phi = d_phiu*flow_u + d_phiv*flow_v;
	 
	 // transformation from spherical to cartesian coordinates (for derotation)
	 *flow_x = -quick_trig_sin(theta)*quick_trig_sin(phi)*flow_theta + quick_trig_cos(theta)*quick_trig_cos(phi)*flow_phi;
	 *flow_y = quick_trig_cos(theta)*quick_trig_sin(phi)*flow_theta + quick_trig_sin(theta)*quick_trig_cos(phi)*flow_phi;
	 *flow_z = -quick_trig_sin(phi)*flow_phi;
}