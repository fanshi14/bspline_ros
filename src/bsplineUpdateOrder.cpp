#include <bspline_ros/bsplineUpdateOrder.h>

void bsplineUpdateOrder::onInit()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("spline_origin_degree", m_origin_deg, 2);
  private_nh.param("spline_update_degree", m_update_deg, 5);
  private_nh.param("is_ts_none",   m_is_TsNone, true);

  m_sub_path_grid_points = m_nh.subscribe<geometry_msgs::PolygonStamped>("/path_gird_points", 1, &bsplineUpdateOrder::pathGridPointsCallback, this);
  m_pub_spline_origin_path = m_nh.advertise<nav_msgs::Path>("spline_path", 1);
  m_pub_spline_update_path = m_nh.advertise<nav_msgs::Path>("spline_update_path", 1);

  m_spline_origin_ptr = new tinyspline::BSpline(m_origin_deg, 3, m_origin_deg+1, TS_CLAMPED);
  m_spline_update_ptr = new tinyspline::BSpline(m_update_deg, 3, m_update_deg+1, TS_CLAMPED);
}

void bsplineUpdateOrder::pathGridPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
  geometry_msgs::PolygonStamped msg_data = *msg;
  bsplineParamInput(&msg_data);
}

bool bsplineUpdateOrder::bsplineParamInput(geometry_msgs::PolygonStamped* msg)
{
  /* Init */
  delete m_spline_origin_ptr;
 
  m_n_controlpts = msg->polygon.points.size() / 2;
  m_n_knots = m_n_controlpts + m_origin_deg + 1;
 
  if (m_n_controlpts <= m_origin_deg){
    ROS_WARN("Control points is LESS than origin degree!");
    std::cout << "Control point num: " << m_n_controlpts << ", degree: "
              << m_origin_deg << "\n";
    return false;
  }

  if (m_is_TsNone)
    m_spline_origin_ptr = new tinyspline::BSpline(m_origin_deg, // degree = order - 1
                                   3, // dim of the data
                                   m_n_controlpts, // control points >= degree+1
                                   TS_NONE);
  else
    m_spline_origin_ptr = new tinyspline::BSpline(m_origin_deg, 3, m_n_controlpts, TS_CLAMPED);

  if (m_is_TsNone){
    m_t0 = msg->polygon.points[0].x;
    m_tn = msg->polygon.points[2*(m_n_controlpts-m_origin_deg)].x;
  }
  else{
    m_t0 = 0.0;
    m_tn = 1.0;
  }

  if (m_is_TsNone){
    /* Manually set knots value if TsNone */
    m_knotpts = m_spline_origin_ptr->knots();
    for (int i = 0; i <= m_origin_deg; ++i){
      m_knotpts[i] = m_t0;
      m_knotpts[m_n_knots-1-i] = m_tn;
    }
    for (int i = 1; i <= m_n_controlpts-1-m_origin_deg; ++i)
      m_knotpts[i+m_origin_deg] = msg->polygon.points[2*i].x;
    m_spline_origin_ptr->setKnots(m_knotpts);
  }

  /* Set control points value */
  m_controlpts = m_spline_origin_ptr->ctrlp();
  for (int i = 0; i < m_n_controlpts; ++i){
    m_controlpts[3*i] = msg->polygon.points[2*i+1].x;
    m_controlpts[3*i+1] = msg->polygon.points[2*i+1].y;
    m_controlpts[3*i+2] = msg->polygon.points[2*i+1].z;
  }

  m_spline_origin_ptr->setCtrlp(m_controlpts);

  originSplinePathDisplay();

  return true;
}

bool bsplineUpdateOrder::bsplineUpdateParamInput()
{
  /* Init */
  delete m_spline_update_ptr;
 
  if (m_n_controlpts <= m_update_deg){
    ROS_WARN("Control points is LESS than update degree!");
    std::cout << "Control point num: " << m_n_controlpts << ", degree: "
              << m_update_deg << "\n";
    return false;
  }

  m_spline_update_ptr = new tinyspline::BSpline(m_update_deg, 3, m_n_controlpts, TS_NONE);

  /* Updated spline knots will be changed, so we manually set knots value in clamped way */
  std::vector<tinyspline::rational> update_knotpts;
  int n_update_knots = m_n_controlpts + m_update_deg + 1;
  update_knotpts = m_spline_update_ptr->knots();

  double start_polygon_time = (m_tn - m_t0) / (m_n_controlpts-m_origin_deg) * m_update_deg / 2.0;
  /* Keep start and end velocity guarantee the requirements, and total time do not change [m_t0, m_tn] */
  for (int i = 0; i <= m_update_deg; ++i){
    update_knotpts[i] = m_t0;
    update_knotpts[n_update_knots-1-i] = m_tn;
  }
  update_knotpts[1+m_update_deg] = m_t0 + start_polygon_time;
  update_knotpts[n_update_knots-2-m_update_deg] = m_tn - start_polygon_time;
  for (int i = 2; i <= m_n_controlpts-2-m_update_deg; ++i)
    update_knotpts[i+m_update_deg] = (update_knotpts[n_update_knots-2-m_update_deg] - update_knotpts[1+m_update_deg]) *i / (m_n_controlpts-m_update_deg-2);
  
  m_spline_update_ptr->setKnots(update_knotpts);
  m_spline_update_ptr->setCtrlp(m_controlpts);
  updateSplinePathDisplay();
  return true;
}

void bsplineUpdateOrder::originSplinePathDisplay()
{
  m_spline_origin_path.header.frame_id = std::string("/world");
  m_spline_origin_path.header.stamp = ros::Time().now();
  float sample_gap = 0.05f;
  m_spline_origin_path.poses.clear();
  int n_sample;
  if (m_is_TsNone)
    n_sample = int((m_tn-m_t0) / sample_gap);
  else
    n_sample = int(1.0f / sample_gap);
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = m_spline_origin_path.header;
  pose_stamped.pose.orientation.x = 0.0f;
  pose_stamped.pose.orientation.y = 0.0f;
  pose_stamped.pose.orientation.z = 0.0f;
  pose_stamped.pose.orientation.w = 1.0f;

  for (int i = 0; i <= n_sample; ++i){
    std::vector<tinyspline::rational> result = m_spline_origin_ptr->evaluate(i*sample_gap).result();
    pose_stamped.pose.position.x = result[0];
    pose_stamped.pose.position.y = result[1];
    pose_stamped.pose.position.z = result[2];
    m_spline_origin_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_origin_path.publish(m_spline_origin_path);
}

void bsplineUpdateOrder::updateSplinePathDisplay()
{
  m_spline_update_path.header.frame_id = std::string("/world");
  m_spline_update_path.header.stamp = ros::Time().now();
  float sample_gap = 0.05f;
  m_spline_update_path.poses.clear();
  int n_sample;
  n_sample = int((m_tn-m_t0) / sample_gap);
  
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = m_spline_update_path.header;
  pose_stamped.pose.orientation.x = 0.0f;
  pose_stamped.pose.orientation.y = 0.0f;
  pose_stamped.pose.orientation.z = 0.0f;
  pose_stamped.pose.orientation.w = 1.0f;

  for (int i = 0; i <= n_sample; ++i){
    std::vector<tinyspline::rational> result = m_spline_update_ptr->evaluate(i*sample_gap).result();
    pose_stamped.pose.position.x = result[0];
    pose_stamped.pose.position.y = result[1];
    pose_stamped.pose.position.z = result[2];
    m_spline_update_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_update_path.publish(m_spline_update_path);
}

void bsplineUpdateOrder::getDerive()
{
  m_spline_origin_derive = m_spline_origin_ptr->derive();
  m_spline_origin_2nd_derive = m_spline_origin_derive;
}

std::vector<double> bsplineUpdateOrder::evaluate(double t)
{
  std::vector<tinyspline::rational> result = m_spline_origin_ptr->evaluate(t).result();
  std::vector<double> res_d;
  res_d.push_back(result[0]);
  res_d.push_back(result[1]);
  res_d.push_back(result[2]);
  return res_d;
}

std::vector<double> bsplineUpdateOrder::evaluateDerive(double t)
{
  std::vector<tinyspline::rational> result = m_spline_origin_derive.evaluate(t).result();
  std::vector<double> res_d;
  res_d.push_back(result[0]);
  res_d.push_back(result[1]);
  res_d.push_back(result[2]);
  return res_d;
}

void bsplineUpdateOrder::getUpdateSplineDerive()
{
  m_spline_update_derive = m_spline_update_ptr->derive();
  m_spline_update_2nd_derive = m_spline_update_derive.derive();
}

std::vector<double> bsplineUpdateOrder::evaluateUpdateSpline(double t, int order)
{
  std::vector<tinyspline::rational> result;
  if (order == 1)
    result = m_spline_update_derive.evaluate(t).result();
  else if (order == 2)
    result = m_spline_update_2nd_derive.evaluate(t).result();
  else
    result = m_spline_update_ptr->evaluate(t).result();
  
  std::vector<double> res_d;
  res_d.push_back(result[0]);
  res_d.push_back(result[1]);
  res_d.push_back(result[2]);
  return res_d;
}

std::vector<double> bsplineUpdateOrder::evaluateOriginSpline(double t, int order)
{
  std::vector<tinyspline::rational> result;
  if (order == 1)
    result = m_spline_origin_derive.evaluate(t).result();
  else if (order == 2)
    result = m_spline_origin_2nd_derive.evaluate(t).result();
  else
    result = m_spline_origin_ptr->evaluate(t).result();
  
  std::vector<double> res_d;
  res_d.push_back(result[0]);
  res_d.push_back(result[1]);
  res_d.push_back(result[2]);
  return res_d;
}
