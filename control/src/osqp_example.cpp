/// \file
/// \brief Basic OSQP example

#include <geometry_msgs/Point.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "osqp.h"

int main(int argc, char **argv) {
  ROS_INFO("STARTING NODE: osqp_example");

  ros::init(argc, argv, "osqp_example_node");  // register the node on ROS
  ros::NodeHandle nh;                          // get a handle to ROS
  ros::NodeHandle nh_("~");                    // get a handle to ROS

  ros::Publisher osqp_pub = nh.advertise<geometry_msgs::Point>("success", 1);

  geometry_msgs::Point min_point;

  // OSQP Example Start: https://osqp.org/docs/examples/setup-and-solve.html
  // Load problem data
  c_float P_x[3] = {
      4.0,
      1.0,
      2.0,
  };
  c_int P_nnz = 3;
  c_int P_i[3] = {
      0,
      0,
      1,
  };
  c_int P_p[3] = {
      0,
      1,
      3,
  };
  c_float q[2] = {
      1.0,
      1.0,
  };
  c_float A_x[4] = {
      1.0,
      1.0,
      1.0,
      1.0,
  };
  c_int A_nnz = 4;
  c_int A_i[4] = {
      0,
      1,
      0,
      2,
  };
  c_int A_p[3] = {
      0,
      2,
      4,
  };
  c_float l[3] = {
      1.0,
      0.0,
      0.0,
  };
  c_float u[3] = {
      1.0,
      0.7,
      0.7,
  };
  c_int n = 2;
  c_int m = 3;

  // Exitflag
  c_int exitflag = 0;

  // Workspace structures
  OSQPWorkspace *work;
  OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
  OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));

  // Populate data
  if (data) {
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
    data->q = q;
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
    data->l = l;
    data->u = u;
  }

  // Define solver settings as default
  if (settings) {
    osqp_set_default_settings(settings);
    settings->alpha = 1.0;  // Change alpha parameter
  }

  // Setup workspace
  exitflag = osqp_setup(&work, data, settings);

  // Solve Problem
  osqp_solve(work);

  // Cleanup
  if (data) {
    if (data->A) c_free(data->A);
    if (data->P) c_free(data->P);
    c_free(data);
  }
  if (settings) c_free(settings);

  // OSQP Example End

  min_point.x = work->solution->x[0];
  min_point.y = work->solution->x[1];

  ROS_INFO("Calculated min: [%f, %f]", min_point.x, min_point.y);

  while (ros::ok()) {
    ros::spinOnce();
    osqp_pub.publish(min_point);
  }

  return exitflag;
}
