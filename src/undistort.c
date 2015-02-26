#include "undistort.h"
//fix header!

//write documentation, add a function to create lookup table?
void cam2world(float point3D[3], float point2D[2])
{
 float invdet  = 1.0f/(AFF_C-AFF_D*AFF_E); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 float xp = invdet*(    (point2D[0] - WIDTH_C) - AFF_D*(point2D[1] - HEIGHT_C) );
 float yp = invdet*( -AFF_E*(point2D[0] - WIDTH_C) + AFF_C*(point2D[1] - HEIGHT_C) );
  
 float r   = sqrtf(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 float zp  = POLY[0];
 float r_i = 1.0f;
 int i;
 
 for (i = 1; i < LENGTH_POLY; i++)
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

uint8_t derotate_of_sphere(float point3D0[3],float point3D1[3],float motion[3],float wx,float wy,  float wz){
motion[0]=point3D1[0]-point3D0[0]-(wy*point3D0[2]-point3D0[1]*wz);
motion[1]=point3D1[1]-point3D0[1]-(point3D0[0]*wz-wx*point3D0[2]);
motion[2]=point3D1[2]-point3D0[2]-(wx*point3D0[1]-point3D0[0]*wy);
return 0;
}

uint8_t point3d_to_angles(float point3D[3], float angles[2]){
  angles[1]=asinf(point3D[2]);
  angles[0]=atan2f(point3D[1]/cosf(angles[1]),point3D[0]/cosf(angles[1]));
  return 0;
}