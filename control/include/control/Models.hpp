#pragma once
/// \file
/// \brief Dynamic Models: Differential Drive.

#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>

#include "control/utilities.hpp"

namespace control {
namespace models {
using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;

/**
 * @brief Differential Drive Model.
 *
 */
class DiffDrive {
 public:
  /**
   * @brief Construct a new Diff Drive object which creates the following:
   *
   * m_a: A matrix (xdot = Ax + Bu).
   * m_b: B matrix (xdot = Ax + Bu). Depends on heading angle.
   * m_gradient: EOM first derivative (to be set by TrajMPC).
   * m_hessian: EOM second derivative (to be set by TrajMPC).
   * m_linear_constraint_matrix: container for linear constraints in QP problem.
   * m_lower_bound, m_upper_bound: bounds for constraints in QP problem.
   *
   * @param wheel_radius Differential Drive robot's wheel radius in m.
   * @param wheel_base Differential Drive robot's wheelbase in m.
   * @param abs_max_wheel_vel Maximum Wheel Velocity (absolute) in rad/s.
   */
  DiffDrive(const double& wheel_radius, const double& wheel_base,
            const double& abs_max_wheel_vel)
      : m_wheel_radius(wheel_radius),
        m_wheel_base(wheel_base),
        m_abs_max_wheel_vel(abs_max_wheel_vel) {
    // DOI: 10.2478/auseme-2018-0002 Model Predictive Control of a
    // Differential-Drive Mobile Robot Samir BOUZOUALEGH 1 , El-Hadi GUECHI 1
    // and Ridha KELAIAIA 2

    // A matrix is identity since no actuation = no movement.
    m_a = MatrixXd::Identity(3, 3);
  }

  /**
   * @brief Returns the number of variables in this optimization.
   *
   * @param mpcWindow Timesteps for which we perform MPC.
   */
  int num_variables(const int& mpcWindow) {
    return m_a.rows() * (mpcWindow + 1) + m_b.cols() * mpcWindow;
  }

  /**
   * @brief Returns the number of constraints in this optimization.
   *
   * @param mpcWindow Timesteps for which we perform MPC.
   */
  int num_constraints(const int& mpcWindow) {
    return 2 * m_a.rows() * (mpcWindow + 1) + m_b.cols() * mpcWindow;
  }

  /**
   * @brief
   *
   * @return Eigen::SparseMatrix<double>&
   */
  Eigen::SparseMatrix<double>& hessian() { return m_hessian; }

  /**
   * @brief
   *
   * @return VectorXd&
   */
  VectorXd& gradient() { return m_gradient; }

  /**
   * @brief
   *
   * @return MatrixXd&
   */
  MatrixXd& linear_constraint_matrix() { return m_linear_constraint_matrix; }

  /**
   * @brief
   *
   * @return VectorXd&
   */
  VectorXd& lower_bound() { return m_lower_bound; }

  /**
   * @brief
   *
   * @return VectorXd&
   */
  VectorXd& upper_bound() { return m_upper_bound; }

  /**
   * @brief
   *
   * @return MatrixXd&
   */
  MatrixXd a() { return m_a; }

  /**
   * @brief
   *
   * @param theta Robot heading in radians.
   * @return MatrixXd
   */
  MatrixXd b(const MatrixXd& state) {
    // B matrix is a function of heading and robot params - see mppi node.
    const double& theta = state(2);

    /**
     * r: wheel radius
     * wb: wheel base
     * u0: left wheel speed
     * u1: right wheel speed
     * th: robot heading
     * | 0.5rcos(th), 0.5rcos(th) |     | u0 |
     * | 0.5rsin(th), 0.5rsin(th) |  X  |    |
     * | -r/wb, r/wb              |     | u1 |
     */

    m_b << 0.5 * m_wheel_radius * std::cos(theta),
        0.5 * m_wheel_radius * std::cos(theta),
        0.5 * m_wheel_radius * std::sin(theta),
        0.5 * m_wheel_radius * std::sin(theta), -m_wheel_radius / m_wheel_base,
        m_wheel_radius / m_wheel_base;

    return m_b;
  }

 private:
  double m_wheel_radius;
  double m_wheel_base;
  double m_abs_max_wheel_vel;
  Eigen::Matrix<double, 3, 3> m_a;
  Eigen::Matrix<double, 3, 2> m_b;
  VectorXd m_gradient;
  Eigen::SparseMatrix<double> m_hessian;
  MatrixXd m_linear_constraint_matrix;
  VectorXd m_lower_bound, m_upper_bound;
};
}  // namespace models
}  // namespace control