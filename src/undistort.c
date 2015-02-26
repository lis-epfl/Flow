#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "settings.h"
#include "utils.h"
#include "undistort.h"


/**
 * @brief Uses the camera model from Scaramuzza's camera calibration toolbox to project 2D points on a unit-sphere
 *
 * @param point2D point in the image plane
 * @param point3D returned point on the unit sphere
 */
void cam2world(float point2D[2],float point3D[3])
{
 float invdet  = 1.0f/(global_data.param[PARAM_CAM_AFF_C]-global_data.param[PARAM_CAM_AFF_D]*global_data.param[PARAM_CAM_AFF_E]); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 float xp = invdet*(    (point2D[0] - global_data.param[PARAM_CAM_CENTER_COL]) - global_data.param[PARAM_CAM_AFF_D]*(point2D[1] - global_data.param[PARAM_CAM_CENTER_ROW]) );
 float yp = invdet*( -global_data.param[PARAM_CAM_AFF_E]*(point2D[0] - global_data.param[PARAM_CAM_CENTER_COL]) + global_data.param[PARAM_CAM_AFF_C]*(point2D[1] - global_data.param[PARAM_CAM_CENTER_ROW]) );
  
 float r   = sqrtf(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 float zp  = POLY[0];
 float r_i = 1.0f;
 int i;
 
 for (i = 1; i < global_data.param[PARAM_CAM_LENGTH_POLY]; i++)
 {
   r_i *= r;
   zp  += r_i*POLY[i];
 }
 
 //normalize to unit norm
 float invnorm = 1.0f/sqrtf( xp*xp + yp*yp + zp*zp );
 
 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp; 
 point3D[2] = invnorm*zp;
}

/**
 * @brief Derotates optic flow from 2 3D corresponding points
 *
 * @param point3D0 point at time t0
 * @param point3D0 point at time t1
 * @param motion returns the rotation compensated motion vector
 * @param wx rotation rate around wx
 * @param wy rotation rate around wy
 * @param wz rotation rate around wz
 */
uint8_t derotate_sphere(float point3D0[3],float point3D1[3],float motion[3],float wx,float wy,  float wz){
motion[0]=point3D1[0]-point3D0[0]-(wy*point3D0[2]-point3D0[1]*wz);
motion[1]=point3D1[1]-point3D0[1]-(point3D0[0]*wz-wx*point3D0[2]);
motion[2]=point3D1[2]-point3D0[2]-(wx*point3D0[1]-point3D0[0]*wy);
return 0;
}

/**
 * @brief Returns azimuth and elevation from 3D point coordinates on a unit sphere
 *
 * @param point3D 3D coordinates of the point
 * @param angles returned azimuth and elevation
 */
uint8_t point3d_to_angles(float point3D[3], float angles[2]){
  angles[1]=asinf(point3D[2]);
  angles[0]=atan2f(point3D[1]/cosf(angles[1]),point3D[0]/cosf(angles[1]));
  return 0;
}