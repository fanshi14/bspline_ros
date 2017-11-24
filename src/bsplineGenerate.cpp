#include <bspline_ros/bsplineGenerate.h>

bsplineGenerate::bsplineGenerate(ros::NodeHandle nh, ros::NodeHandle nhp, std::string spline_path_pub_topic_name){
  m_nh = nh;
  m_nhp = nhp;
  m_debug = true;
  m_first_display_flag = false;

  m_pub_spline_path = m_nh.advertise<nav_msgs::Path>(spline_path_pub_topic_name, 1);
  m_pub_reconstructed_path_markers = m_nh.advertise<visualization_msgs::MarkerArray>("reconstructed_path_markers", 1);
}

void bsplineGenerate::pathGridPointsCallback(const bspline_ros::ControlPointsConstPtr& msg)
{
  if (m_debug)
    ROS_INFO("\nReceive control points from topic.\n");
  bspline_ros::ControlPoints msg_data = *msg;
  bsplineParamInput(&msg_data);
}

void bsplineGenerate::bsplineParamInput(bspline_ros::ControlPoints* msg)
{
  /* Init */
  if (m_first_display_flag){
    controlPolygonDisplay(0);
    delete m_spline_ptr;
  }
  else
    m_first_display_flag = true;

  m_is_TsNone = !msg->is_uniform;
  m_deg = msg->degree;
  m_dim = msg->dim;
  m_n_controlpts = msg->num;
  m_n_knots = m_n_controlpts + m_deg + 1;
  if (m_debug)
    std::cout << "B-spline input control points number: " << m_n_controlpts << "\n";

  if (m_n_controlpts <= m_deg){
    ROS_WARN("Control points is LESS than degree!");
    if (m_debug)
      std::cout << "Control point num: " << m_n_controlpts << ", degree: "
                << m_deg << "\n";
    return;
  }
  if (m_is_TsNone)
    m_spline_ptr = new tinyspline::BSpline(m_deg, // degree = order - 1
                                           m_dim, // dim of the data
                                           m_n_controlpts, // control points >= degree+1
                                           TS_NONE);
  else
    m_spline_ptr = new tinyspline::BSpline(m_deg, m_dim, m_n_controlpts, TS_CLAMPED);

  if (msg->knots.data.empty()){
    m_t0 = 0.0;
    m_tn = 1.0;
  }
  else{
    m_t0 = msg->knots.data[0];
    m_tn = msg->knots.data[m_n_knots - 1];
  }
  if (m_debug)
    std::cout << "Time region: ["<< m_t0 << ", " << m_tn << "]\n";

  m_knotpts = m_spline_ptr->knots();
  if (m_is_TsNone){
    /* Manually set knots value if TsNone */
    for (int i = 0; i < m_n_knots; ++i){
      m_knotpts[i] = msg->knots.data[i];
    }
    m_spline_ptr->setKnots(m_knotpts);
  }

  if (m_debug){
    std::cout << "Start position: ";
    for (int i = 0; i < m_dim; ++i)
      std::cout << msg->control_pts.data[i] << ", ";
    std::cout << "\nEnd position: ";
    for (int i = 0; i < m_dim; ++i)
      std::cout << msg->control_pts.data[i + (m_n_controlpts-1) * m_dim] << ", ";
    std::cout << "\n";
  }

  /* Set control points value */
  m_controlpts = m_spline_ptr->ctrlp();
  for (int i = 0; i < m_n_controlpts * m_dim; ++i)
    m_controlpts[i] = msg->control_pts.data[i];

  /* Debug */
  // if (m_debug){
  //   std::cout << "[check knots]: \n";
  //   for (int i = 0; i < m_n_knots; ++i)
  //       std::cout << m_knotpts[i] << ", ";
  //   std::cout << "\n";
  // }

  m_spline_ptr->setCtrlp(m_controlpts);

  splinePathDisplay();
  controlPolygonDisplay(1);
  if (m_debug)
    std::cout << "Spline display finished.\n";
}

void bsplineGenerate::splinePathDisplay()
{
  nav_msgs::Path spline_path;
  spline_path.header.frame_id = std::string("/world");
  spline_path.header.stamp = ros::Time().now();
  float sample_gap = 0.02f;
  spline_path.poses.clear();
  int n_sample;
  if (m_is_TsNone)
    n_sample = int((m_tn-m_t0) / sample_gap);
  else
    n_sample = int(1.0f / sample_gap);
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = spline_path.header;
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
    spline_path.poses.push_back(pose_stamped);
  }
  m_pub_spline_path.publish(spline_path);
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
