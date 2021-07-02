#pragma once
/// \file
/// \brief Model Predictive Control for Trajectory Following

#include <OsqpEigen/OsqpEigen.h>

#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>

#include "control/utilities.hpp"

namespace control {
using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector2d;
using Eigen::VectorXd;

/**
 * @brief Model Predictive Control for Trajectory Following.
 *
 * Model: Contains the Dynamic Model for the optimization.
 *  METHODS:
 *    num_variables(): return number of variables in the problem.
 *    num_constraints(): return number of constraints in the problem.
 *    gradient(): return a reference to the dynamic model's gradient.
 *    hessian(): return a reference to the dynamic model's hessian.
 *    linear_constraint_matrix(): return the dynamic model's constraint matrix.
 *    lower_bound(): return a reference to the lower bound constraint vector.
 *    upper_bound(): return a reference to the upper bound constraint vector.
 *    a(): return A matrix (xdot = Ax + Bu)
 *    b(current state): return B matrix (xdot = Ax + Bu)
 *
 * Action: Container for returning minimizing commands.
 *
 * Problem: Contains the problem formulation for the optimization.
 *  METHODS:
 *    Q(): return state cost Matrix (LQR formulation).
 *    R(): return action cost Matrix (LQR formulation).
 *    xMax(): return upper state constraint bound.
 *    xMin(): return lower state constraint bound.
 *    uMax(): return upper action constraint bound.
 *    uMin(): return lower action constraint bound.
 *    x0(): return initial state for optimization.
 *    xRef(): return reference state for optimization.
 *
 */
template <typename Model, typename Problem, typename State, typename Action>
class TrajMPC {
 public:
  /**
   * @brief Constructor which takes in dynamic model.
   *
   * @param model contains the xdot = Ax + Bu transition, as well as the
   * constraints.
   */
  TrajMPC(const Model& model, const int& mpcWindow,
          const double& solve_threshold = 1.0e-12,
          const int& max_solve_steps = 50)
      : m_model(model),
        m_mpcWindow(mpcWindow),
        m_solve_threshold(solve_threshold),
        m_max_solve_steps(max_solve_steps) {}

  /**
   * @brief Set up OSQP Solver with relevant parameters.
   *
   * @param problem Container for problem information: Cost Matrices, State and
   * Control Limits, Reference and Initial Trajectory.
   * @return size_t solver setup result. 0 is success, 1 is failure.
   */
  size_t setup(const Problem& problem) {
    // Grab problem data.
    const auto& Q = problem.Q();
    const auto& R = problem.R();
    const auto& xMax = problem.xMax();
    const auto& xMin = problem.xMin();
    const auto& uMax = problem.uMax();
    const auto& uMin = problem.uMin();
    const auto& x0 = problem.x0();
    const auto& xRef = problem.xRef();

    // Grab model data. (xdot = Ax + Bu)
    const auto a = m_model.a();
    const auto b = m_model.b(x0);

    // cast the MPC problem as QP problem
    control::utilities::castMPCToQPHessian<Q.rows(), R.rows()>(
        Q, R, m_mpcWindow, m_model.hessian());
    control::utilities::castMPCToQPGradient<Q.rows(), xRef.rows(), xRef.cols()>(
        Q, xRef, m_mpcWindow, m_model.gradient());
    control::utilities::castMPCToQPConstraintMatrix<a.rows(), a.cols(),
                                                    b.rows(), b.cols()>(
        a, b, m_mpcWindow, m_model.linear_constraint_matrix());
    control::utilities::castMPCToQPConstraintVectors<xMax.rows(), uMax.rows(),
                                                     x0.rows()>(
        xMax, xMin, uMax, uMin, x0, m_mpcWindow, m_model.lower_bound(),
        m_model.upper_bound());

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(m_model.num_variables(m_mpcWindow));
    solver.data()->setNumberOfConstraints(m_model.num_constraints(m_mpcWindow));
    if (!solver.data()->setHessianMatrix(m_model.hessian())) return 1;
    if (!solver.data()->setGradient(m_model.gradient())) return 1;
    if (!solver.data()->setLinearConstraintsMatrix(
            m_model.linear_constraint_matrix()))
      return 1;
    if (!solver.data()->setLowerBound(m_model.lower_bound())) return 1;
    if (!solver.data()->setUpperBound(m_model.upper_bound())) return 1;

    // instantiate the solver
    if (!solver.initSolver()) return 1;

    m_solver = solver;
    return 0;
  }

  /**
   * @brief Assign the optimal control action around the desired reference and
   * return an exit code for this OSQP solver iteration.
   *
   * @param problem Container for problem information: Cost Matrices, State and
   * Control Limits, Reference and Initial Trajectory.
   * @param action Container for assigning optimal control action.
   * @return OSQP exit code.
   */
  size_t solve(const Problem& problem, Action* action) {
    // Grab updated problem parameters
    const auto& Q = problem.Q();
    auto& x0 = problem.x0();
    const auto& xRef = problem.xRef();

    // Grab solver
    auto& solver = m_solver;

    // Update Gradient based on new xRef
    control::utilities::castMPCToQPGradient<Q.rows(), xRef.rows(), xRef.cols()>(
        Q, xRef, m_mpcWindow, m_model.gradient());
    if (!solver.updateGradient(m_model.gradient())) return 1;
    // Update Constraint Vector based on new initial state
    control::utilities::updateConstraintVectors<x0.rows()>(
        x0, m_model.lower_bound(), m_model.upper_bound());
    if (!solver.updateBounds(m_model.lower_bound(), m_model.upper_bound()))
      return 1;

    // controller input and QPSolution vector
    Eigen::Vector4d ctr;
    // Assign zero control to action in case of failure.
    action->cmd = ctr;
    Eigen::VectorXd QPSolution;

    // Exceeded max steps.
    size_t exit_code = -1;

    double obj_val = std::numeric_limits<double>::max();

    for (int i = 0; i < m_max_solve_steps; i++) {
      // solve the QP problem
      if (!solver.solve()) return 1;

      // get the controller input
      QPSolution = solver.getSolution();
      ctr = QPSolution.block(problem.xMax().rows() * (m_mpcWindow + 1), 0,
                             problem.uMax().rows(), 1);

      // save data into file
      // auto x0Data = x0.data();

      // propagate the model - TODO: RK4
      x0 = m_model.a() * x0 + m_model.b(x0) * ctr;

      // update the constraint bound
      control::utilities::updateConstraintVectors<x0.rows()>(
          x0, m_model.lower_bound(), m_model.upper_bound());
      if (!solver.updateBounds(m_model.lower_bound(), m_model.upper_bound()))
        return 1;

      /// Get the optimal objective value, can be used to exit early.
      /// https://osqp.org/docs/interfaces/cc++?highlight=osqpworkspace#_CPPv413OSQPWorkspace
      const auto new_obj_val = solver.workspace()->info->obj_val;

      if (std::abs(new_obj_val - obj_val) < m_solve_threshold) {
        obj_val = new_obj_val;
        break;
      }
      obj_val = new_obj_val;
    }
    action->cmd = ctr;
    return 0;
  }

 private:
  /// MPC Horizon.
  int m_mpcWindow;
  /// Cost change threshold for stopping optimization.
  double m_solve_threshold;
  /// Maximum number of optimization steps before giving up.
  int m_max_solve_steps;
  /// Dynamic Model used to solve optimization problem.
  Model m_model;
  /// OSQP Solver with Eigen C++ Wrapper.
  OsqpEigen::Solver m_solver;
};
}  // namespace control