#include <bspline_ros/bsplineGenerate.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tinyspline");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  bsplineGenerate bspline_vis(nh, nhp);
  bspline_vis.onInit();

  ros::spin();
  return 0;
}

