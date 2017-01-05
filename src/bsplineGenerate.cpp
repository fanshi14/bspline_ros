#include <bspline_ros/bsplineGenerate.h>

void bsplineGenerate::onInit()
{
  m_sub_path_grid_points = m_nh.subscribe<geometry_msgs::PolygonStamped>("/path_gird_points", 1, &bsplineGenerate::pathGridPointsCallback, this);
  m_pub_spline_path = m_nh.advertise<nav_msgs::Path>("spline_path", 1);

  m_deg = 3;
  m_spline_ptr = new ts::BSpline(3, 3, 7, TS_NONE);
}

void bsplineGenerate::pathGridPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
  ROS_INFO("Polygon data comes.");
  std::cout << "Start point is " << msg->polygon.points[1].x <<' ' << msg->polygon.points[1].y << ' ' << msg->polygon.points[1].z<<"\n";

  m_n_control_pts = msg->polygon.points.size() / 2;
  if (m_n_control_pts*2 != msg->polygon.points.size())
    ROS_WARN("PolygonStamped msg is not complicated, whose number is not even.");
  // degree = 3, 3d cubic spline, number of control points
  delete m_spline_ptr;
  m_spline_ptr = new ts::BSpline(m_deg, 3, m_n_control_pts, TS_NONE);
  //m_spline_ptr = new ts::BSpline(5, 3, m_n_control_pts, TS_CLAMPED);
  m_controlpts = m_spline_ptr->ctrlp();
  m_knotpts = m_spline_ptr->knots();
  for (int i = 0; i < m_deg; ++i){
    m_knotpts[i] = msg->polygon.points[0].x;
    m_knotpts[m_n_control_pts-1-i] = msg->polygon.points[2*m_n_control_pts-2].x;
  }
  for (int i = 0; i < m_n_control_pts; ++i){
    m_knotpts[i+m_deg] = msg->polygon.points[2*i].x;
    m_controlpts[3*i] = msg->polygon.points[2*i+1].x;
    m_controlpts[3*i+1] = msg->polygon.points[2*i+1].y;
    m_controlpts[3*i+2] = msg->polygon.points[2*i+1].z;
  }
  m_spline_ptr->setCtrlp(m_controlpts);
  m_spline_ptr->setKnots(m_knotpts);

  m_spline_path.header = msg->header;
  splinePathDisplay();
}

void bsplineGenerate::splinePathDisplay()
{
  float sample_gap = 0.1f;
  m_spline_path.poses.clear();
  int n_sample = int(m_knotpts[m_n_control_pts-1] / sample_gap);
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = m_spline_path.header;
  pose_stamped.pose.orientation.x = 0.0f;
  pose_stamped.pose.orientation.y = 0.0f;
  pose_stamped.pose.orientation.z = 0.0f;
  pose_stamped.pose.orientation.w = 1.0f;
  std::cout << "Num of samples: " << n_sample << " \n";
  for (int i = 0; i < n_sample; ++i){
    std::vector<ts::rational> result = m_spline_ptr->evaluate(i*sample_gap).result();
    pose_stamped.pose.position.x = result[0];
    pose_stamped.pose.position.y = result[1];
    pose_stamped.pose.position.z = result[2];
    m_spline_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_path.publish(m_spline_path);
}
