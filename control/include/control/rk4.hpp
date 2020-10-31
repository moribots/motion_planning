#ifndef RK4_HPP
#define RK4_HPP
/// \file
/// \brief 4th Order Runge Kutta method for Integration

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <functional>

namespace control
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;
  using Eigen::Vector2d;
  using Eigen::Ref;


  /// \brief Integrator
  class RK4
  {
    public:
      /// \brief Integrator constructor with fixed step size
      /// \param dt - time step for integration
      RK4(const double & dt);

      /// \brief Register a generic ODE without a control signal. updates the last input by ref (non-const)
      /// \param ode - function to integrate
      void registerODE(std::function<void(const Ref<VectorXd>, Ref<VectorXd>)> ode);

      /// \brief Register a generic ODE with a control signal. updates the last input by ref (non-const)
      /// \param ode - function to integrate
      void registerODE(std::function<void(const Ref<VectorXd>, const Ref<VectorXd>, Ref<VectorXd>)> ode);

      /// \brief Solve ODE or system of ODEs without a control signal using RK4
      /// \param x0 - initial condition(state)
      /// \param horizon - final time of integration
      MatrixXd solve(const Ref<VectorXd> x0, const double & horizon);

      /// \brief Solve ODE or system of ODEs with a control signal using RK4
      /// \param x0 - initial condition(state)
      /// \param u - control signal (must be of length equal to number of rk4 iterations => horizon/step)
      /// \param horizon - final time of integration
      MatrixXd solve(const Ref<VectorXd> x0, const Ref<MatrixXd> u, const double & horizon);
    private:
      /// \brief Perform one iteration of rk4 without a control signal
      /// x_t[out] - update the current state x_t
      void integrate(Ref<VectorXd> x_t);

      /// \brief Perform one iteration of rk4 with a control signal
      /// param u_t - control vector
      /// x_t[out] - update the current state x_t
      void integrate(Ref<VectorXd> x_t, const Ref<VectorXd> u_t);

      // ODE without control args: x(t), xdot[out]. updates the last input by ref (non-const)
      std::function<void(const Ref<VectorXd>, Ref<VectorXd>)> func;

      // ODE with control args: x(t), u(t), xdot[out]. updates the last input by ref (non-const)
      std::function<void(const Ref<VectorXd>, const Ref<VectorXd>, Ref<VectorXd>)> func_ctrl;

      // time step
      double dt;
  };
}
#endif