#include <iostream>
/*we need set the following flag to disable c++11 for linking the tinyspline */
#define TINYSPLINE_DISABLE_CXX11_FEATURES
#include <tinysplinecpp.h>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class bsplineUpdateOrder
{
public:
  ts::BSpline* m_spline_origin_ptr;
  ts::BSpline m_spline_origin_derive;
  ts::BSpline m_spline_origin_2nd_derive;
  ts::BSpline* m_spline_update_ptr;
  ts::BSpline m_spline_update_derive;
  ts::BSpline m_spline_update_2nd_derive;
  std::vector<ts::rational> m_controlpts;
  std::vector<ts::rational> m_knotpts;
  int m_n_controlpts;
  int m_n_knots;
  int m_origin_deg;
  int m_update_deg;
  nav_msgs::Path m_spline_origin_path;
  nav_msgs::Path m_spline_update_path;
  bool m_is_TsNone;
  float m_t0, m_tn;

  ros::NodeHandle m_nh;
  ros::Subscriber m_sub_path_grid_points;
  ros::Publisher m_pub_spline_origin_path;
  ros::Publisher m_pub_spline_update_path;

  void onInit();
  void pathGridPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg);
  bool bsplineParamInput(geometry_msgs::PolygonStamped* msg);
  bool bsplineUpdateParamInput();
  void originSplinePathDisplay();
  void updateSplinePathDisplay();  
  void getDerive();  
  void getUpdateSplineDerive();
  std::vector<double> evaluate(double t);
  std::vector<double> evaluateDerive(double t);
  std::vector<double> evaluateUpdateSpline(double t, int order=0);
  std::vector<double> evaluateOriginSpline(double t, int order=0);
};
