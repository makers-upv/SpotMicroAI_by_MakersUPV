#include "inverseKinematics.hpp"

void IK2D :: begin(float l1, float l2){
  L1 = l1;
  L2 = l2;
}

void IK2D :: setOriginOffset(float x, float y){
  xOffset = x;
  yOffset = y;
}

void IK2D :: coordToAngles(float x, float y, float* q1, float* q2){
  x += xOffset;
  y += yOffset;

  *q2 = PI - acosf((L1*L1 + L2*L2 - x*x - y*y )/(2*L1*L2));
  *q1 = atanf(x/-y)-atanf((L2*sin(*q2))/(L1+L2*cos(*q2)));

  *q2 *= 180/PI;
  *q1 *= 180/PI;
}


void IK3D :: begin(float l1, float l2, float l3){
  L1 = l1;
  L2 = l2;
  L3 = l3;
}

void IK3D :: setOriginOffset(float x, float y, float z){
  xOffset = x;
  yOffset = y;
  zOffset = z;
}

void IK3D :: coordToAngles(float x, float y, float z, float* q1, float* q2, float* q3){
  x += xOffset;
  y += yOffset;
  z += zOffset;

  /*
  *q1 = ;
  *q2 = ;
  *q3 = ;
  */
}