/// \file
/// \brief 4th Order Runge Kutta method for Integration

#include <iostream>
#include "control/rk4.hpp"

namespace control
{

RK4::RK4(const double & dt) : dt(dt)
{
}


void RK4::registerODE(std::function<void(const Ref<VectorXd>, Ref<VectorXd>)> ode)
{
  func = ode;
}


void RK4::registerODE(std::function<void(const Ref<VectorXd>, const Ref<VectorXd>, Ref<VectorXd>)> ode)
{
  func_ctrl = ode;
}


MatrixXd RK4::solve(const Ref<VectorXd> x0, const double & horizon)
{
  // catch undeclared fcn
  if(!func)
  {
    std::cerr << "No ODE Function!" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Initial State
  VectorXd state = x0;
  // Solver iterations
  const auto num_samples = static_cast<int>(horizon/dt);
  // Solution Trajectory Shape
  MatrixXd trajectory(state.rows(), num_samples);

  // Integrate for all samples
  for(int i = 0; i < num_samples; i++)
  {
    // Integrate once (pass state by reference using Eigen::Ref<>)
    integrate(state);
    // Store next state in trajectory vector
    trajectory.col(i) = state;
  }

  return trajectory;
}


MatrixXd RK4::solve(const Ref<VectorXd> x0, const Ref<MatrixXd> u, const double & horizon)
{
  // catch undeclared fcn
  if(!func_ctrl)
  {
    std::cerr << "No ODE Function!" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Initial State
  VectorXd state = x0;
  // Solver iterations
  const auto num_samples = static_cast<int>(horizon/dt);
  // Solution Trajectory Shape
  MatrixXd trajectory(state.rows(), num_samples);

  // Integrate for  all samples
  for(int i = 0; i < num_samples; i++)
  {
    // Get Control Vector
    VectorXd u_t = u.col(i);
    // Integrate once (pass state by reference using Eigen::Ref<>)
    integrate(state, u_t);
    // Store next state in trajectory vector
    trajectory.col(i) = state;
  }

  return trajectory;
}


void RK4::integrate(Ref<VectorXd> x_t)
{
  // Pass input as reference to save memory

  // Initialize at zero
  VectorXd k1 = VectorXd::Zero(x_t.size());
  VectorXd k2, k3, k4, temp_pass;
  k2 = k3 = k4 = temp_pass = k1;

  // RK4 method without control signal

  // get k1 by ref
  func(x_t, k1);
  // get k2 by ref
  temp_pass = x_t + dt*(0.5*k1);
  func(temp_pass, k2);
  // get k3 by ref
  temp_pass = x_t + dt*(0.5*k2);
  func(temp_pass, k3);
  // get k4 by ref
  temp_pass = x_t + dt*k3;
  func(temp_pass, k4);

  // update (init + increment)
  x_t += (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}


void RK4::integrate(Ref<VectorXd> x_t, const Ref<VectorXd> u_t)
{
  // Pass input as reference to save memory

  // Initialize at zero
  VectorXd k1 = VectorXd::Zero(x_t.size());
  VectorXd k2, k3, k4, temp_pass;
  k2 = k3 = k4 = temp_pass = k1;

  // RK4 method with control signal

  // get k1 by ref
  func_ctrl(x_t, u_t, k1);
  // get k2 by ref
  temp_pass = x_t + dt*(0.5*k1);
  func_ctrl(temp_pass, u_t, k2);
  // get k3 by ref
  temp_pass = x_t + dt*(0.5*k2);
  func_ctrl(temp_pass, u_t, k3);
  // get k4 by ref
  temp_pass = x_t + dt*k3;
  func_ctrl(temp_pass, u_t, k4);

  // update (init + increment)
  x_t += (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}




}