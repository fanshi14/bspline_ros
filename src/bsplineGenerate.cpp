#include <bspline_ros/bsplineGenerate.h>

void bsplineGenerate::onInit()
{
  m_sub_path_grid_points = m_nh.subscribe<geometry_msgs::PolygonStamped>("/path_gird_points", 1, &bsplineGenerate::pathGridPointsCallback, this);
  m_pub_spline_path = m_nh.advertise<nav_msgs::Path>("spline_path", 1);

  m_deg = 3;
  isTsNone = false;
  m_spline_ptr = new ts::BSpline(3, 3, 7, TS_NONE);
}

void bsplineGenerate::pathGridPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
  ROS_INFO("Polygon data comes.");
  std::cout << "Start point is " << msg->polygon.points[1].x <<' ' << msg->polygon.points[1].y << ' ' << msg->polygon.points[1].z<<"\n";

  m_n_controlpts = msg->polygon.points.size() / 2;
  std::cout << "Input control points number: " << m_n_controlpts << "\n";
  // degree = 3, 3d cubic spline, number of control points
  delete m_spline_ptr;
  if (isTsNone)
    m_spline_ptr = new ts::BSpline(m_deg, 3, m_n_controlpts, TS_NONE);
  else{
    if (m_n_controlpts <= m_deg)
      m_spline_ptr = new ts::BSpline(m_deg, 3, m_deg+1, TS_CLAMPED);
    else
      m_spline_ptr = new ts::BSpline(m_deg, 3, m_n_controlpts, TS_CLAMPED);
  }

  if (isTsNone){
    m_knotpts = m_spline_ptr->knots();
    for (int i = 0; i < m_deg; ++i){
      m_knotpts[i] = msg->polygon.points[0].x;
      m_knotpts[m_n_controlpts-1-i] = msg->polygon.points[2*m_n_controlpts-2].x;
    }
    for (int i = 0; i < m_n_controlpts; ++i)
      m_knotpts[i+m_deg] = msg->polygon.points[2*i].x;
    m_spline_ptr->setKnots(m_knotpts);
  }

  m_t0 = msg->polygon.points[0].x;
  m_tn = msg->polygon.points[2*m_n_controlpts-2].x;
  m_controlpts = m_spline_ptr->ctrlp();
  for (int i = 0; i < m_n_controlpts; ++i){
    m_controlpts[3*i] = msg->polygon.points[2*i+1].x;
    m_controlpts[3*i+1] = msg->polygon.points[2*i+1].y;
    m_controlpts[3*i+2] = msg->polygon.points[2*i+1].z;
  }

  if (m_n_controlpts <= m_deg)
    completeControlPoints();

  m_spline_ptr->setCtrlp(m_controlpts);

  m_spline_path.header = msg->header;
  splinePathDisplay();
}

void bsplineGenerate::splinePathDisplay()
{
  float sample_gap = 0.02f;
  m_spline_path.poses.clear();
  int n_sample;
  if (isTsNone)
    n_sample = int((m_tn-m_t0) / sample_gap);
  else
    n_sample = int(1.0f / sample_gap);
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = m_spline_path.header;
  pose_stamped.pose.orientation.x = 0.0f;
  pose_stamped.pose.orientation.y = 0.0f;
  pose_stamped.pose.orientation.z = 0.0f;
  pose_stamped.pose.orientation.w = 1.0f;
  std::cout << "Time region: ["<< m_t0 << ", " << m_tn << "]\n";
  std::cout << "Num of samples: " << n_sample << " \n";

  //ts::BSpline beziers = m_spline_ptr->derive().toBeziers();

  for (int i = 0; i < n_sample; ++i){
    std::vector<ts::rational> result = m_spline_ptr->evaluate(i*sample_gap).result();
    pose_stamped.pose.position.x = result[0];
    pose_stamped.pose.position.y = result[1];
    pose_stamped.pose.position.z = result[2];
    m_spline_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_path.publish(m_spline_path);
}

void bsplineGenerate::completeControlPoints()
{
  switch (m_n_controlpts){
  case 1:
    ROS_WARN("Only one control point, should already arrive, may be potential BUG.");
    break;
  case 2:
    m_controlpts[3*m_deg] = m_controlpts[3*m_n_controlpts-3];
    m_controlpts[3*m_deg+1] = m_controlpts[3*m_n_controlpts-2];
    m_controlpts[3*m_deg+2] = m_controlpts[3*m_n_controlpts-1];
    // If only have two control points, add other control points along their connection line and seperate in average way.
    for (int i = 1; i <= m_deg-1; ++i){
      for (int j = 0; j < 3; ++j)
        m_controlpts[3*i+j] = float(i)/float(m_deg) * (m_controlpts[3*m_deg+j]-m_controlpts[j]);
    }
    break;
  default:
    // Move existed control points to the most end.
    for (int i = 1; i < m_n_controlpts; ++i){
      for (int j = 0; j < 3; ++j)
        m_controlpts[3*(i+m_deg+1-m_n_controlpts)+j] = m_controlpts[3*i+j];
    }
    // Insert new control points between first and second control point in average way.
    for (int i = 1; i <= m_deg+1-m_n_controlpts; ++i){
      for (int j = 0; j < 3; ++j)
        m_controlpts[3*i+j] = float(i)/float(m_deg+2-m_n_controlpts) * (m_controlpts[3*(m_deg+2-m_n_controlpts)+j]-m_controlpts[j]);
    }
    if (m_n_controlpts >= 4)
      ROS_WARN("Now only have strategy for degree 3. If it is more than 3, just simply insert completion control points between first and second point.");
  }
  m_n_controlpts = m_deg + 1;
}
