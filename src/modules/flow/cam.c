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
#include "flow2.h"

void cam2world(float *xp, float *yp, float *zp, float u, float v, cam_model *cam)
{
	 float *pol    = cam->pol;
	 float xc      = cam->xc;
	 float yc      = cam->yc;
	 float c       = cam->c;
	 float d       = cam->d;
	 float e       = cam->e;
	 uint8_t length_pol = cam->length_pol;

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

void flow2world(float *flow_x, float *flow_y, float *flow_z, float xp, float yp, float zp, float flow_u, float flow_v, cam_model *cam)
{
	 float *pol    = cam->pol;
	 float c       = cam->c;
	 float d       = cam->d;
	 float e       = cam->e;
	 uint8_t length_pol = cam->length_pol;

	 float invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

	 // transformation from rectangular to spherical coordinates
	 float r   = maths_fast_sqrt(SQR(xp) + SQR(yp));
	 float theta = quick_trig_atan2(yp,xp);
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
	 *flow_y = -quick_trig_sin(theta)*quick_trig_sin(phi)*flow_theta + quick_trig_cos(theta)*quick_trig_cos(phi)*flow_phi; // x and y were swapped
	 *flow_x = quick_trig_cos(theta)*quick_trig_sin(phi)*flow_theta + quick_trig_sin(theta)*quick_trig_cos(phi)*flow_phi; // x and y were swapped
	 *flow_z = -quick_trig_sin(phi)*flow_phi;
}

void fast_flow2world(float *flow_x,
						float *flow_y,
						float *flow_z,
						float xp1,
						float yp1,
						float zp1,
						float u1,
						float v1,
						float flow_u,
						float flow_v,
						cam_model *cam)
{
	 // add optic-flow to pixel coordinates (u1, v1)
	 float u2 = u1 + flow_u;
	 float v2 = v1 + flow_v;

	 // define new coordinates
	 float xp2;
	 float yp2;
	 float zp2;

	 // map to 3d world
	 cam2world(&xp2,  &yp2,  &zp2,  u2,  v2, cam);

	 // project on unit sphere
	 normalize(&xp2, &yp2, &zp2);

	 // compute optic-flow considering small angles
	 *flow_x = xp2 - xp1;
	 *flow_y = yp2 - yp1;
	 *flow_z = zp2 - zp1;
}
