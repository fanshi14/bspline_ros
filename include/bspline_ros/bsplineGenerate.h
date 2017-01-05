#include <iostream>
#include <tinysplinecpp.h>
#include <geometry_msgs/Point32.h>

class bsplineGenerate
{
public:
  ts::BSpline m_spline;
  std::vector<ts::rational> m_controlpts;

  void onInit();
};
