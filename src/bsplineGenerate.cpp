#include <bspline_ros/bsplineGenerate.h>

void bsplineGenerate::onInit()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("spline_degree", m_default_deg, 2);
  private_nh.param("is_ts_none",   m_is_TsNone, true);

  m_sub_path_grid_points = m_nh.subscribe<geometry_msgs::PolygonStamped>("/path_gird_points", 1, &bsplineGenerate::pathGridPointsCallback, this);
  m_pub_spline_path = m_nh.advertise<nav_msgs::Path>("spline_path", 1);

  m_spline_ptr = new ts::BSpline(m_default_deg, 3, m_default_deg+1, TS_CLAMPED);
}

void bsplineGenerate::pathGridPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
  /* Init */
  delete m_spline_ptr;
  m_deg = m_default_deg;

  m_n_controlpts = msg->polygon.points.size() / 2;
  m_n_knots = m_n_controlpts + m_deg + 1;
  std::cout << "B-spline input control points number: " << m_n_controlpts << "\n";
  if (m_n_controlpts <= m_deg){
    ROS_WARN("Control points is LESS than degree!");
    std::cout << "Control point num: " << m_n_controlpts << ", degree: "
              << m_deg << "\n";
    m_deg = m_n_controlpts - 1;
  }

  if (m_is_TsNone)
    m_spline_ptr = new ts::BSpline(m_deg, // degree = order - 1
                                   3, // dim of the data
                                   m_n_controlpts, // control points >= degree+1
                                   TS_NONE);
  else
    m_spline_ptr = new ts::BSpline(m_deg, 3, m_n_controlpts, TS_CLAMPED);

  m_t0 = msg->polygon.points[0].x;
  //m_tn = msg->polygon.points[2*(m_n_controlpts-m_deg-1)].x;
  m_tn = msg->polygon.points[2*(m_n_controlpts-m_deg)].x;
  std::cout << "Time region: ["<< m_t0 << ", " << m_tn << "]\n";

  if (m_is_TsNone){
    /* Manually set knots value if TsNone */
    m_knotpts = m_spline_ptr->knots();
    for (int i = 0; i <= m_deg; ++i){
      m_knotpts[i] = m_t0;
      m_knotpts[m_n_knots-1-i] = m_tn;
    }
    for (int i = 1; i <= m_n_controlpts-1-m_deg; ++i)
      m_knotpts[i+m_deg] = msg->polygon.points[2*i].x;
    m_spline_ptr->setKnots(m_knotpts);
  }

  /* Set control points value */
  m_controlpts = m_spline_ptr->ctrlp();
  for (int i = 0; i < m_n_controlpts; ++i){
    m_controlpts[3*i] = msg->polygon.points[2*i+1].x;
    m_controlpts[3*i+1] = msg->polygon.points[2*i+1].y;
    m_controlpts[3*i+2] = msg->polygon.points[2*i+1].z;
  }

  /* Debug */
  // std::cout << "[check knots]: \n";
  // for (int i = 0; i < m_n_controlpts+m_deg; ++i){
  //   std::cout << m_knotpts[i] << ", ";
  // }
  // std::cout << "\n";

  m_spline_ptr->setCtrlp(m_controlpts);

  m_spline_path.header = msg->header;
  splinePathDisplay();
}

void bsplineGenerate::splinePathDisplay()
{
  float sample_gap = 0.02f;
  m_spline_path.poses.clear();
  int n_sample;
  if (m_is_TsNone)
    n_sample = int((m_tn-m_t0) / sample_gap);
  else
    n_sample = int(1.0f / sample_gap);
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = m_spline_path.header;
  pose_stamped.pose.orientation.x = 0.0f;
  pose_stamped.pose.orientation.y = 0.0f;
  pose_stamped.pose.orientation.z = 0.0f;
  pose_stamped.pose.orientation.w = 1.0f;

  //ts::BSpline beziers = m_spline_ptr->derive().toBeziers();

  for (int i = 0; i <= n_sample; ++i){
    std::vector<ts::rational> result = m_spline_ptr->evaluate(i*sample_gap).result();
    pose_stamped.pose.position.x = result[0];
    pose_stamped.pose.position.y = result[1];
    pose_stamped.pose.position.z = result[2];
    m_spline_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_path.publish(m_spline_path);
}
