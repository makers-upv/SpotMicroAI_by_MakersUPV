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
  y += yOffset;
  x += xOffset;

  *q1 = atan2f((L1*L1*L1*x*x + L1*x*x*(-L2*L2 + x*x + y*y) + y*sqrtf(-(L1*L1*x*x*(L1*L1*L1*L1 + (-L2*L2 + x*x + y*y)*(-L2*L2 + x*x + y*y) - 2*L1*L1*(L2*L2 + x*x + y*y)))))/(L1*L1*x*(x*x + y*y)), (-(L1*L1*L1*y) - L1*y*(-L2*L2 + x*x + y*y) + sqrtf(-(L1*L1*x*x*(L1*L1*L1*L1 + (-L2*L2 + x*x + y*y)*(-L2*L2 + x*x + y*y) - 2*L1*L1*(L2*L2 + x*x + y*y)))))/(L1*L1*(x*x + y*y)));
  *q2 = atan2f(sqrtf(-(L1*L1*x*x*(L1*L1*L1*L1 + (-L2*L2 + x*x + y*y)*(-L2*L2 + x*x + y*y) - 2*L1*L1*(L2*L2 + x*x + y*y))))/(L1*L1*L2*x), (-L1*L1 - L2*L2 + x*x + y*y)/(L1*L2));

  //*q1 = atan2f((L1*L1*L1*x*x + L1*x*x*(-L2*L2 + x*x + y*y) - y*sqrtf(-(L1*L1*x*x*(L1*L1*L1*L1 + (-L2*L2 + x*x + y*y)*(-L2*L2 + x*x + y*y) - 2*L1*L1*(L2*L2 + x*x + y*y)))))/(L1*L1*x*(x*x + y*y)),(-(L1*L1*L1*y) + L1*L2*L2*y - L1*x*x*y - L1*y*y*y - sqrtf(-(L1*L1*x*x*(L1*L1*L1*L1 + (-L2*L2 + x*x + y*y)*(-L2*L2 + x*x + y*y) - 2*L1*L1*(L2*L2 + x*x + y*y)))))/(L1*L1*(x*x + y*y)));
  //*q2 = atan2f(-(sqrtf(-(L1*L1*x*x*(L1*L1*L1*L1 + (-L2*L2 + x*x + y*y)*(-L2*L2 + x*x + y*y) - 2*L1*L1*(L2*L2 + x*x + y*y))))/(L1*L1*L2*x)), (-L1*L1 - L2*L2 + x*x + y*y)/(L1*L2));
  //*q2 = acosf((x*x + y*y - L1*L1 -L2*L2)/(2*L1*L2));
  //*q1 = atanf(x/(-y))-atanf((L2*sin(*q2))/(L1+L2*cos(*q2)));

  *q2 *= 180/PI;
  *q1 *= 180/PI;
}
