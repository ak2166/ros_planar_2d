/* Author: Anthony A. Kolodinzkski
** This C++ class holds methods that will calculate the inverse kinematics of the 2 DOF arm
**/
#include "Planar2D.h"
#include <math.h>

float* Planar2D::calculateIK(float x, float y){
      float l = sqrt(link_one^2 + link_two^2);
      float gamma = arccos((l^2 + link_one^2 - link_two^2)/(2*link_one*link_two));
      float theta_one = arctan(x/y)-gamma;
      float theta_two = arctan((y-(link_one*sin(theta_one)))/(x-(link_one*cos(theta_one))))-theta_one;
      float angles[2] = {theta_one, theta_two};
      return angles;

   }
}

Planar2D::Planar2D(float l1, float l2){
      link_one = l1;
      link_two = l2;
}
