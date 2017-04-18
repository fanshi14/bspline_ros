#include <bspline_ros/bsplineUpdateOrder.h>
#include <std_msgs/Empty.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <iostream>
#define PI 3.14159265

void updateSplineTestCallback(const std_msgs::Empty msg)
{
  bsplineUpdateOrder spline;
  spline.onInit();
  int n_ctrl_pts = 10;
  while (1){
    geometry_msgs::PolygonStamped* polygon_ptr = new geometry_msgs::PolygonStamped();
    double end_time;
    end_time = 1.0 * (n_ctrl_pts - 2);
    std::vector<double> ctrl_pts;
    // double ctrl_pts[24] = {
    //   -8.0, -2.0, 0.0,
    //   -5.0, 5.0, 0.0,
    //   -3.0, -1.0, 0.0,
    //   0.0, 5.0, 0.0,
    //   3.5, -2.0, 0.0,
    //   7.0, 4.0, 0.0,
    //   10.0, -2.0, 0.0,
    //   17.0, 3.0, 0.0};
    ctrl_pts.push_back(0.0); ctrl_pts.push_back(0.0); ctrl_pts.push_back(0.0); // start point: (0, 0, 0), but not the first control point, which will be corrected in the end of the code
    double max_vel = 10.0, max_acc = 3.0;
    srand (time(NULL));
    double cur_vel = rand() / (double)RAND_MAX * max_vel;

    /* Make n_ctrl_pts a random value between [7, 10] */
    n_ctrl_pts += 1;
    if (n_ctrl_pts > 10)
      n_ctrl_pts = 7;
    
    double cur_ang = rand() / (double)RAND_MAX * 2.0 * PI;
    // Add second control point
    ctrl_pts.push_back(cur_vel * cos(cur_ang)); ctrl_pts.push_back(cur_vel * sin(cur_ang)); ctrl_pts.push_back(0.0);
    for (int i = 2; i <= n_ctrl_pts-1; ++i){
      double center_x, center_y, start_x, start_y;
      start_x = ctrl_pts[3*(i-1)];
      start_y = ctrl_pts[3*(i-1)+1];
      center_x = 2.0*start_x - ctrl_pts[3*(i-2)];
      center_y = 2.0*start_y - ctrl_pts[3*(i-2)+1];
      while (1){
	double delta_vel = rand() / (double)RAND_MAX * max_acc;
        cur_ang = rand() / (double)RAND_MAX * 2.0 * PI;
	double cur_x, cur_y;
	cur_x = delta_vel * cos(cur_ang) + center_x;
	cur_y = delta_vel * sin(cur_ang) + center_y;
	if (pow(cur_x - start_x, 2.0) + pow(cur_y - start_y, 2.0) < pow(max_vel, 2.0)){
	  ctrl_pts.push_back(cur_x); ctrl_pts.push_back(cur_y); ctrl_pts.push_back(0.0);
	  break;
	}
      }
    }
    // Correct first and last point
    for (int i = 0; i < 3; ++i){
      ctrl_pts[i] = (ctrl_pts[i] + ctrl_pts[i+3]) / 2.0;
      ctrl_pts[3*n_ctrl_pts-i-1] = (ctrl_pts[3*n_ctrl_pts-i-1] + ctrl_pts[3*n_ctrl_pts-i-4]) / 2.0;
    }
    
    for (int i = 0; i < n_ctrl_pts; ++i){
      geometry_msgs::Point32 control_point, time_point;
      time_point.x = 1.0 * i;
      polygon_ptr->polygon.points.push_back(time_point);
      control_point.x = ctrl_pts[i*3];
      control_point.y = ctrl_pts[i*3+1];
      control_point.z = ctrl_pts[i*3+2];
      polygon_ptr->polygon.points.push_back(control_point);
    }
    spline.bsplineParamInput(polygon_ptr);
    spline.bsplineUpdateParamInput();

    // examine velocity and acceleration
    spline.getDerive();
    spline.getUpdateSplineDerive();

    // test
    // std::vector<double> temp = spline.evaluateDerive(0.0);
    // std::cout << temp[0] << ", " << temp[1] << "\n";
    // temp = spline.evaluateUpdateSpline(0.0, 1);
    // std::cout << temp[0] << ", " << temp[1] << "\n\n";


    // test for original acceleration
    for (int i = 0; i <= (int)((n_ctrl_pts-2)*1.0/0.2); ++i){
      double neighbor_t;
      if (i == (int)((n_ctrl_pts-2)*1.0/0.2))
      	neighbor_t = 0.2 * i - 0.01;
      else
      	neighbor_t = 0.2 * i + 0.01;
      std::vector<double> origin_vel = spline.evaluateOriginSpline(0.2 * i, 1);
      std::vector<double> origin_neighbor_vel = spline.evaluateOriginSpline(neighbor_t, 1);
      std::vector<double> origin_acc;
      for (int j = 0; j < 3; ++j)
	origin_acc.push_back((origin_neighbor_vel[j] - origin_vel[j]) / 0.01);
      if (pow(origin_acc[0], 2.0) + pow(origin_acc[1], 2.0) > pow(max_acc, 2.0) + 0.2){
      	ROS_ERROR("ORIGIN Acceleration is over upperbound.");
	std::cout << 0.2*i << ": " <<  origin_acc[0] << ", " << origin_acc[1] << ". " << sqrt(pow(origin_acc[0], 2.0) + pow(origin_acc[1], 2.0)) << "\n";
      }
    }
    
    bool is_vel_flag = false;
    // !! for 5 order, we multiply with 2.5 in time domain
    for (int i = 0; i <= (int)((n_ctrl_pts-5)*2.5/0.2); ++i){
      std::vector<double> vel = spline.evaluateUpdateSpline(0.2 * i, 1);

      // examine velocity
      if (pow(vel[0], 2.0) + pow(vel[1], 2.0) > pow(max_vel, 2.0) + 0.2){
	if (!is_vel_flag){ // Only output one time for same bspline function
	  ROS_ERROR("Velocity is over upperbound.");
	  is_vel_flag = true;
	  std::cout << "Order is: " << n_ctrl_pts << "\n";
	}
	std::cout << 0.2 * i << ": " << vel[0] << ", " << vel[1] << ". " << sqrt(pow(vel[0], 2.0) + pow(vel[1], 2.0)) << "\n";
	std::vector<double> origin_vel = spline.evaluateOriginSpline(0.2 * i, 1);
	std::cout << "Origin: " <<  origin_vel[0] << ", " << origin_vel[1] << ". " << sqrt(pow(origin_vel[0], 2.0) + pow(origin_vel[1], 2.0)) << "\n";
	for (int j = 0; j < 3*n_ctrl_pts; ++j){
	  std::cout << ctrl_pts[j] << ", ";
	}
	std::cout << "\n\n";
	// directly break, do not display the whole outlier points
	break;
      }

      // examine acceleration
      double neighbor_t;
      if (i == (int)((n_ctrl_pts-5)*2.5/0.2))
      	neighbor_t = 0.2 * i - 0.01;
      else
      	neighbor_t = 0.2 * i + 0.01;
      std::vector<double> neighbor_vel = spline.evaluateUpdateSpline(neighbor_t, 1);
      std::vector<double> acc;
      for (int j = 0; j < 3; ++j)
      	acc.push_back((neighbor_vel[j] - vel[j]) / 0.01);
      if (pow(acc[0], 2.0) + pow(acc[1], 2.0) > pow(max_acc, 2.0) + 0.2){
      	ROS_ERROR("Acceleration is over upperbound.");
      	std::cout << "Order is: " << n_ctrl_pts << "\n";
      	std::cout << 0.2 * i << ": " << acc[0] << ", " << acc[1] << ". " << sqrt(pow(acc[0], 2.0) + pow(acc[1], 2.0)) << "\n";
      	for (int j = 0; j < 3*n_ctrl_pts; ++j){
      	  std::cout << ctrl_pts[j] << ", ";
      	}
      	std::cout << "\n\n";
      	// directly break, do not display the whole outlier points
      	break;
      }
       
    }
    
    usleep(1000000);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "update_bspline");
  ros::NodeHandle m_nh;
  ros::Subscriber m_sub_update_spline_test_start_flag;
  m_sub_update_spline_test_start_flag = m_nh.subscribe<std_msgs::Empty>("/update_spline_test_start_flag", 1, updateSplineTestCallback);

  ros::spin();
  return 0;
}

