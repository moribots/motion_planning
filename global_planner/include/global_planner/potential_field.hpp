#ifndef POTENTIAL_FIELD_INCLUDE_GUARD_HPP
#define POTENTIAL_FIELD_INCLUDE_GUARD_HPP
/// \file
/// \brief Potential Field planning library.
#include <global_planner/global_planner.hpp>

namespace global
{
    // Used to store Obstacle vertex coordinates
    using rigid2d::Vector2D;
    using map::Vertex;
    using map::Cell;
    using map::Index;
    using map::Obstacle;

    /// \brief Potential Field (online-capable) planner
    class PotentialField : public GlobalPlanner
    {
    public:
        /// \brief constructor to initialize the Potential Field Planner
        // param obstacles_: the obstacle vectors containing bocked vertices in the map
        // param eta_: the termination condition for gradient descent. Also the descent direction
        // param zeta_: the attractive factor multiplier
        // param ada_: the repulsive factor multiplier
        // param d_thresh_: threshold for transition between quadratic and conic potential
        // param Q_thresh_: range of influence of obstacle
        PotentialField(const std::vector<Obstacle> & obstacles_, const double & eta_, const double & ada_,
                       const double & zeta_, const double & d_thresh_, const double & Q_thresh_);

        // \brief Calculates Attractive Gradient towards goal
        // \param cur_pos: the robot's current position, used for gradient computation
        // \param goal: the goal position, used for gradient computation
        // \returns: Vector2D containing gradient in x and y dimensions
        Vector2D AttractiveGradient(const Vector2D & cur_pos, const Vector2D & goal);

        // \brief Calculates Repulsive Gradient (cummulative) based on all obstacles
        // \param cur_pos: the robot's current position, used for gradient computation
        // \param obs: vector of Obstacle used for gradient computation. may be full map or reduced for
        // simulated visibility
        // \returns: Vector2D containing gradient in x and y dimensions
        Vector2D RepulsiveGradient(const Vector2D & cur_pos, const std::vector<Obstacle> & obs);

        // \brief find the closest point between a robot position and an obstacle
        // \param cur_pos: the robot's current position, used for gradient computation
        // \param vertices: vertices of an obstacle that we will use to find the closest point
        // \returns: Vector2D containing closest point's x,y coordinates
        Vector2D FindClosestPoint(const Vector2D & cur_pos, const std::vector<Vector2D> & vertices);

        // \brief returns the multi-dimensional norm of our gradient
        // \param partials: the partial gradients in each dimension
        // \returns: the norm
        double MultiDimNorm(const std::vector<double> partials);

        // \brief perform Gradient Descent for one step to move closer to the goal
        // \param cur_pos: the robot's current position, used for gradient computation
        // \param goal: the goal position, used for gradient computation
        // \param obs: vector of Obstacle used for gradient computation. may be full map or reduced for
        // simulated visibility
        // \returns: the robot's new x,y coordinates after Gradient Descent
        Vector2D OneStepGD(const Vector2D & cur_pos, const Vector2D & goal, std::vector<Obstacle> & obs);

        // \brief returns whether we have reached the goal
        // \returns: boolean to indicate termination
        bool return_terminate();


    // protected instead of private so that child Class can access
    protected:
        double eta;
        double zeta;
        double ada;
        double d_thresh;
        double Q_thresh;
        bool terminate = false;
    };
}

#endif
