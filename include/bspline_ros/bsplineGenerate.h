#include <iostream>
/*we need set the following flag to disable c++11 for linking the tinyspline */
#define TINYSPLINE_DISABLE_CXX11_FEATURES
#include <tinysplinecpp.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unistd.h>
#include <stdlib.h>
#include <bspline_ros/ControlPoints.h>

class bsplineGenerate
{
public:
  tinyspline::BSpline* m_spline_ptr;
  tinyspline::BSpline m_spline_derive;
  std::vector<tinyspline::rational> m_controlpts;
  std::vector<tinyspline::rational> m_knotpts;
  int m_n_controlpts;
  int m_n_knots;
  int m_deg;
  int m_dim;
  bool m_is_TsNone;
  float m_t0, m_tn;
  bool m_first_display_flag;
  bool m_debug;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_nhp;
  ros::Subscriber m_sub_path_grid_points;
  ros::Publisher m_pub_spline_path;
  ros::Publisher m_pub_reconstructed_path_markers;

  bsplineGenerate(ros::NodeHandle nh, ros::NodeHandle nhp);
  /* onInit() for tinyspline_ros_node or bsplineGenerateLibrary. */
  void onInit();
  void onInit(std::string spline_path_pub_topic_name);
  void pathGridPointsCallback(const bspline_ros::ControlPointsConstPtr& msg);
  void splinePathDisplay();
  void bsplineParamInput(bspline_ros::ControlPoints* msg);
  void getDerive();
  std::vector<double> evaluate(double t);
  std::vector<double> evaluateDerive(double t);
  void controlPolygonDisplay(int mode);
  void arrayConvertToPoint(int id, geometry_msgs::Point& point);
};
