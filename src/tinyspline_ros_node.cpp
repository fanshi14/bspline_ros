#include <bspline_ros/bsplineGenerate.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tinyspline");
  bsplineGenerate bspline_vis;
  bspline_vis.onInit();

  ros::spin();
  return 0;
}

