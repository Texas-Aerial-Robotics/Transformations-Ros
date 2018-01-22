// Transformation between the facedown camera frame and the drone frame


#include <cmath>


const float ALPHA = 31.8244;
const float PIXLES[2] = [640, 480];
const float PI = 3.14159;


double pixel2metric_facedown(double alt, double obj_pix)
{

  double H_p = PIXLES[1];
  double ang = (ALPHA/2.0)*(PI/180.0);
  double H_m = 2.0*tan(ang)*alt;

  double O_m = (obj_pix/H_p)*H_m;
  return O_m;

}
