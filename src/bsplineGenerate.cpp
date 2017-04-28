#include <bspline_ros/bsplineGenerate.h>

void bsplineGenerate::onInit()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("spline_degree", m_default_deg, 2);
  private_nh.param("is_ts_none",   m_is_TsNone, true);

  m_sub_path_grid_points = m_nh.subscribe<geometry_msgs::PolygonStamped>("/path_gird_points", 1, &bsplineGenerate::pathGridPointsCallback, this);
  m_pub_spline_path = m_nh.advertise<nav_msgs::Path>("spline_path", 1);
  m_pub_reconstructed_path_markers = m_nh.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);

  m_first_display_flag = false;
  m_spline_ptr = new tinyspline::BSpline(m_default_deg, 3, m_default_deg+1, TS_CLAMPED);
}


void bsplineGenerate::onInit(int degree, bool isTsNone, std::string spline_path_pub_topic_name)
{
  m_default_deg = degree;
  m_is_TsNone = isTsNone;
  m_pub_spline_path = m_nh.advertise<nav_msgs::Path>(spline_path_pub_topic_name, 1);
  m_pub_reconstructed_path_markers = m_nh.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
  m_first_display_flag = false;
  m_spline_ptr = new tinyspline::BSpline(m_default_deg, 3, m_default_deg+1, TS_CLAMPED);
}

void bsplineGenerate::pathGridPointsCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
  ROS_INFO("\nReceive control points from topic.\n");
  geometry_msgs::PolygonStamped msg_data = *msg;
  bsplineParamInput(&msg_data);
}

void bsplineGenerate::bsplineParamInput(geometry_msgs::PolygonStamped* msg)
{
  /* Init */
  delete m_spline_ptr;
  m_deg = m_default_deg;
  //if (!m_controlpts.empty())
  if (m_first_display_flag){
    controlPolygonDisplay(0);
  }
  else
    m_first_display_flag = true;

  m_n_controlpts = msg->polygon.points.size() / 2;
  m_n_knots = m_n_controlpts + m_deg + 1;
  //std::cout << "B-spline input control points number: " << m_n_controlpts << "\n";
  if (m_n_controlpts <= m_deg){
    ROS_WARN("Control points is LESS than degree!");
    std::cout << "Control point num: " << m_n_controlpts << ", degree: "
              << m_deg << "\n";
    m_deg = m_n_controlpts - 1;
  }

  if (m_is_TsNone)
    m_spline_ptr = new tinyspline::BSpline(m_deg, // degree = order - 1
                                   3, // dim of the data
                                   m_n_controlpts, // control points >= degree+1
                                   TS_NONE);
  else
    m_spline_ptr = new tinyspline::BSpline(m_deg, 3, m_n_controlpts, TS_CLAMPED);

  m_t0 = msg->polygon.points[0].x;
  //m_tn = msg->polygon.points[2*(m_n_controlpts-m_deg-1)].x;
  m_tn = msg->polygon.points[2*(m_n_controlpts-m_deg)].x;
  // std::cout << "Time region: ["<< m_t0 << ", " << m_tn << "]\n";

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

  splinePathDisplay();
  controlPolygonDisplay(1);
  // std::cout << "Spline display finished.\n";
}

void bsplineGenerate::splinePathDisplay()
{
  m_spline_path.header.frame_id = std::string("/world");
  m_spline_path.header.stamp = ros::Time().now();
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

  //tinyspline::BSpline beziers = m_spline_ptr->derive().toBeziers();

  for (int i = 0; i <= n_sample; ++i){
    std::vector<tinyspline::rational> result = m_spline_ptr->evaluate(i*sample_gap).result();
    pose_stamped.pose.position.x = result[0];
    pose_stamped.pose.position.y = result[1];
    pose_stamped.pose.position.z = result[2];
    m_spline_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_path.publish(m_spline_path);
}

void bsplineGenerate::controlPolygonDisplay(int mode){
  int control_points_num = m_n_controlpts;
  //std::cout << "[Display] Control points number: " << control_points_num << "\n";
  int id_cnt = 0;
  visualization_msgs::MarkerArray path_markers;
  visualization_msgs::Marker control_point_marker, line_list_marker, triangle_list_marker;
  control_point_marker.ns = line_list_marker.ns = "control_polygon";
  control_point_marker.header.frame_id = line_list_marker.header.frame_id = std::string("/world");
  control_point_marker.header.stamp = line_list_marker.header.stamp = ros::Time().now();
  if (mode == 1)
    control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::ADD;
  else
    control_point_marker.action = line_list_marker.action = visualization_msgs::Marker::DELETE;

  triangle_list_marker.header = line_list_marker.header;
  triangle_list_marker.action = line_list_marker.action;
  triangle_list_marker.ns = line_list_marker.ns;

  control_point_marker.type = visualization_msgs::Marker::SPHERE;
  line_list_marker.type = visualization_msgs::Marker::LINE_LIST;
  triangle_list_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

  /* triangle edges */
  line_list_marker.id = id_cnt;
  ++id_cnt;
  line_list_marker.scale.x = 0.07;
  line_list_marker.color.r = 0.0;
  line_list_marker.color.g = 1.0;
  line_list_marker.color.b = 0.0;
  line_list_marker.color.a = 0.3;
  geometry_msgs::Point pt;
  arrayConvertToPoint(0, pt);
  line_list_marker.points.push_back(pt);
  arrayConvertToPoint(1, pt);
  line_list_marker.points.push_back(pt);
  for (int i = 2; i < control_points_num; ++i){
    arrayConvertToPoint(i-2, pt);
    line_list_marker.points.push_back(pt);
    arrayConvertToPoint(i, pt);
    line_list_marker.points.push_back(pt);
    arrayConvertToPoint(i-1, pt);
    line_list_marker.points.push_back(pt);
    arrayConvertToPoint(i, pt);
    line_list_marker.points.push_back(pt);
  }
  path_markers.markers.push_back(line_list_marker);

  /* triangle vertices */
  for (int i = 0; i < control_points_num; ++i){
    control_point_marker.id = id_cnt;
    ++id_cnt;
    control_point_marker.pose.position.x = m_controlpts[3*i];
    control_point_marker.pose.position.y = m_controlpts[3*i+1];
    control_point_marker.pose.position.z = m_controlpts[3*i+2];
    control_point_marker.pose.orientation.x = 0.0;
    control_point_marker.pose.orientation.y = 0.0;
    control_point_marker.pose.orientation.z = 0.0;
    control_point_marker.pose.orientation.w = 1.0;
    if (i == 0 || i == control_points_num-1){
      control_point_marker.scale.x = 0.3;
      control_point_marker.scale.y = 0.3;
      control_point_marker.scale.z = 0.3;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(control_point_marker);
    }
    else{
      control_point_marker.scale.x = 0.2;
      control_point_marker.scale.y = 0.2;
      control_point_marker.scale.z = 0.2;
      control_point_marker.color.a = 1;
      control_point_marker.color.r = 0.0f;
      control_point_marker.color.g = 1.0f;
      control_point_marker.color.b = 0.0f;
      path_markers.markers.push_back(control_point_marker);
    }
  }

  /* triangle list */
  triangle_list_marker.scale.x = 1.0;
  triangle_list_marker.scale.y = 1.0;
  triangle_list_marker.scale.z = 0.0;
  triangle_list_marker.color.a = 0.3;
  srand (time(NULL));
  for (int i = 2; i < control_points_num; ++i){
    triangle_list_marker.id = id_cnt;
    ++id_cnt;
    triangle_list_marker.color.r = rand() / (double)RAND_MAX * 1.0;
    triangle_list_marker.color.g = rand() / (double)RAND_MAX * 1.0;
    triangle_list_marker.color.b = rand() / (double)RAND_MAX * 1.0;
    arrayConvertToPoint(i-2, pt);
    triangle_list_marker.points.push_back(pt);
    arrayConvertToPoint(i-1, pt);
    triangle_list_marker.points.push_back(pt);
    arrayConvertToPoint(i, pt);
    triangle_list_marker.points.push_back(pt);
    path_markers.markers.push_back(triangle_list_marker);
    triangle_list_marker.points.clear();
  }

  m_pub_reconstructed_path_markers.publish(path_markers);
}

void bsplineGenerate::getDerive()
{
  m_spline_derive = m_spline_ptr->derive();
}

std::vector<double> bsplineGenerate::evaluate(double t)
{
  std::vector<tinyspline::rational> result = m_spline_ptr->evaluate(t).result();
  std::vector<double> res_d;
  res_d.push_back(result[0]);
  res_d.push_back(result[1]);
  res_d.push_back(result[2]);
  return res_d;
}

std::vector<double> bsplineGenerate::evaluateDerive(double t)
{
  std::vector<tinyspline::rational> result = m_spline_derive.evaluate(t).result();
  std::vector<double> res_d;
  res_d.push_back(result[0]);
  res_d.push_back(result[1]);
  res_d.push_back(result[2]);
  return res_d;
}

void bsplineGenerate::arrayConvertToPoint(int id, geometry_msgs::Point& point){
  point.x = m_controlpts[3*id];
  point.y = m_controlpts[3*id+1];
  point.z = m_controlpts[3*id+2];
}
