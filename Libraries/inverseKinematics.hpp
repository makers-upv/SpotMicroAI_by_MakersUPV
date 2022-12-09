#include <math.h>

#define PI 3.1415926

class IK2D {
  private:
    float L1;
    float L2;
    float yOffset = 0;
    float xOffset = 0;

  public:
    void begin(float l1, float l2);
    void setOriginOffset(float x, float y);
    void coordToAngles(float x, float y, float* q1, float* q2);
};