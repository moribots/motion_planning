/// \file
/// \brief Basic OSQP example

#include <geometry_msgs/Point.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "control/utilities.hpp"

void setDynamicsMatrices(Eigen::Matrix<double, 12, 12> &a,
                         Eigen::Matrix<double, 12, 4> &b) {
  a << 1., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0.1, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.1, 0., 0., 0.,
      0.0488, 0., 0., 1., 0., 0., 0.0016, 0., 0., 0.0992, 0., 0., 0., -0.0488,
      0., 0., 1., 0., 0., -0.0016, 0., 0., 0.0992, 0., 0., 0., 0., 0., 0., 1.,
      0., 0., 0., 0., 0., 0.0992, 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
      0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
      0., 0., 0., 1., 0., 0., 0., 0.9734, 0., 0., 0., 0., 0., 0.0488, 0., 0.,
      0.9846, 0., 0., 0., -0.9734, 0., 0., 0., 0., 0., -0.0488, 0., 0., 0.9846,
      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.9846;

  b << 0., -0.0726, 0., 0.0726, -0.0726, 0., 0.0726, 0., -0.0152, 0.0152,
      -0.0152, 0.0152, -0., -0.0006, -0., 0.0006, 0.0006, 0., -0.0006, 0.0000,
      0.0106, 0.0106, 0.0106, 0.0106, 0, -1.4512, 0., 1.4512, -1.4512, 0.,
      1.4512, 0., -0.3049, 0.3049, -0.3049, 0.3049, -0., -0.0236, 0., 0.0236,
      0.0236, 0., -0.0236, 0., 0.2107, 0.2107, 0.2107, 0.2107;
}

void setInequalityConstraints(Eigen::Matrix<double, 12, 1> &xMax,
                              Eigen::Matrix<double, 12, 1> &xMin,
                              Eigen::Matrix<double, 4, 1> &uMax,
                              Eigen::Matrix<double, 4, 1> &uMin) {
  double u0 = 10.5916;

  // input inequality constraints
  uMin << 9.6 - u0, 9.6 - u0, 9.6 - u0, 9.6 - u0;

  uMax << 13 - u0, 13 - u0, 13 - u0, 13 - u0;

  // state inequality constraints
  xMin << -M_PI / 6, -M_PI / 6, -OsqpEigen::INFTY, -OsqpEigen::INFTY,
      -OsqpEigen::INFTY, -1., -OsqpEigen::INFTY, -OsqpEigen::INFTY,
      -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY,
      -OsqpEigen::INFTY;

  xMax << M_PI / 6, M_PI / 6, OsqpEigen::INFTY, OsqpEigen::INFTY,
      OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY,
      OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 12> &Q,
                       Eigen::DiagonalMatrix<double, 4> &R) {
  Q.diagonal() << 0, 0, 10., 10., 10., 10., 0, 0, 0, 5., 5., 5.;
  R.diagonal() << 0.1, 0.1, 0.1, 0.1;
}

int main(int argc, char **argv) {
  ROS_INFO("STARTING NODE: osqp_example");

  ros::init(argc, argv, "osqp_example_node");  // register the node on ROS
  ros::NodeHandle nh;                          // get a handle to ROS
  ros::NodeHandle nh_("~");                    // get a handle to ROS

  // OSQP Eigen Example Start:
  // https://robotology.github.io/osqp-eigen/md_pages_mpc.html

  // set the preview window
  int mpcWindow = 20;

  // allocate the dynamics matrices
  Eigen::Matrix<double, 12, 12> a;
  Eigen::Matrix<double, 12, 4> b;

  // allocate the constraints vector
  Eigen::Matrix<double, 12, 1> xMax;
  Eigen::Matrix<double, 12, 1> xMin;
  Eigen::Matrix<double, 4, 1> uMax;
  Eigen::Matrix<double, 4, 1> uMin;

  // allocate the weight matrices
  Eigen::DiagonalMatrix<double, 12> Q;
  Eigen::DiagonalMatrix<double, 4> R;

  // allocate the initial and the reference state space
  Eigen::Matrix<double, 12, 1> x0;
  Eigen::Matrix<double, 12, 1> xRef;

  // allocate QP problem matrices and vectores
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> linearMatrix;
  Eigen::VectorXd lowerBound;
  Eigen::VectorXd upperBound;

  // set the initial and the desired states
  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  xRef << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // set MPC problem quantities
  setDynamicsMatrices(a, b);
  setInequalityConstraints(xMax, xMin, uMax, uMin);
  setWeightMatrices(Q, R);

  // cast the MPC problem as QP problem
  control::utilities::castMPCToQPHessian<12, 4>(Q, R, mpcWindow, hessian);
  control::utilities::castMPCToQPGradient<12, 12, 1>(Q, xRef, mpcWindow,
                                                     gradient);
  control::utilities::castMPCToQPConstraintMatrix<12, 12, 12, 4>(
      a, b, mpcWindow, linearMatrix);
  control::utilities::castMPCToQPConstraintVectors<12, 4, 12>(
      xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound, upperBound);

  // instantiate the solver
  OsqpEigen::Solver solver;

  // settings
  // solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);

  // set the initial data of the QP solver
  solver.data()->setNumberOfVariables(12 * (mpcWindow + 1) + 4 * mpcWindow);
  solver.data()->setNumberOfConstraints(2 * 12 * (mpcWindow + 1) +
                                        4 * mpcWindow);
  if (!solver.data()->setHessianMatrix(hessian)) return 1;
  if (!solver.data()->setGradient(gradient)) return 1;
  if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
  if (!solver.data()->setLowerBound(lowerBound)) return 1;
  if (!solver.data()->setUpperBound(upperBound)) return 1;

  // instantiate the solver
  if (!solver.initSolver()) return 1;

  // controller input and QPSolution vector
  Eigen::Vector4d ctr;
  Eigen::VectorXd QPSolution;

  // number of iteration steps
  int numberOfSteps = 50;

  double obj_val;

  for (int i = 0; i < numberOfSteps; i++) {
    ROS_INFO("---------------------------------------\nIteration: %d", i);
    // solve the QP problem
    if (!solver.solve()) return 1;

    // get the controller input
    QPSolution = solver.getSolution();
    ctr = QPSolution.block(12 * (mpcWindow + 1), 0, 4, 1);

    // save data into file
    // auto x0Data = x0.data();

    // propagate the model
    x0 = a * x0 + b * ctr;

    /// Get the optimal objective value, can be used to exit early.
    /// https://osqp.org/docs/interfaces/cc++?highlight=osqpworkspace#_CPPv413OSQPWorkspace
    obj_val = solver.workspace()->info->obj_val;

    // update the constraint bound
    control::utilities::updateConstraintVectors<12>(x0, lowerBound, upperBound);
    if (!solver.updateBounds(lowerBound, upperBound)) return 1;
  }

  // OSQP Eigen Example End

  ROS_INFO("Solved with obj val [%f]: Ctrl + C to end.", obj_val);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
