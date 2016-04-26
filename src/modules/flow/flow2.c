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


