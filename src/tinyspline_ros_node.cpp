#include <bspline_ros/bsplineGenerate.h>

bsplineGenerate *bspline_vis;

void controlPtsCallback(const bspline_ros::ControlPointsConstPtr& msg){
  bspline_ros::ControlPoints control_pts = *msg;
  bspline_vis->bsplineParamInput(&control_pts);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tinyspline");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  bspline_vis = new bsplineGenerate(nh, nhp, "/spline_path");

  ros::Subscriber control_pts_sub = nh.subscribe("/path_gird_points", 1, controlPtsCallback);

  ros::spin();
  return 0;
}

