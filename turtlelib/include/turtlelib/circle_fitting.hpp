#ifndef CIRCLE_FITTING_INCLUDE_GUARD_HPP
#define CIRCLE_FITTING_INCLUDE_GUARD_HPP

#include <iosfwd>
#include "turtlelib/diff_drive.hpp"
#include <armadillo>

namespace turtlelib {
    /// \brief The circle used for fitting
    struct Circle {
        /// \brief the x-coordinate of the circle
        double x = 0.0;
        /// \brief the y-coordinate of the circle
        double y = 0.0;
        /// \brief the radius of the circle
        double r = 0.0;
    };

    /// \brief fit a circle within a cluster
    /// \param cluster - a cluster of points
    /// \return detected circle
    Circle fit_circle(std::vector<turtlelib::Vector2D> cluster);
}

#endif