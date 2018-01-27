// Transformation between the facedown camera frame and the drone frame

#include <iostream>
#include <cmath>


const float ALPHA = 31.8244;
const float BETA  = 40.4497;
const float PIXLES[2] = {640, 480};
const float PI = 3.14159;

using namespace std;

double pixel2metric_facedown(double alt, double obj_pix)
{

  double H_p  = PIXLES[1];
  double angy = (ALPHA/2.0)*(PI/180.0);
  double H_m  = 2.0*tan(angy)*alt;

  double O_my = (obj_pix/H_p)*H_m;

  double W_p  = PIXLES[0];
  double angx = (beta/2.0)*(PI/180.0);
  double W_m  = 2.0*tan(angx)*alt;

  double O_mx = (obj_pix/W_p)*W_m;
  return double O_m[2] = {O_mx, O_my};
}

int main()
{
  // purely for testing purposes
	double yPos = pixel2metric_facedown(1, 1);
	cout << yPos << endl;

	return 0; 
}
