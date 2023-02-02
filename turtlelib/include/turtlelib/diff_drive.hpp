#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of a differential drive robot


#include <iosfwd>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib {
    /// \brief the radius of the wheels for the turtlebot
    constexpr double wheel_radius = 0.033;
    /// \brief the distance between the wheels for the turtlebot
    constexpr double track_width = 0.16;

    /// \brief The angles of the robot's wheels
    struct WheelAngle {
        /// \brief the angle of the left wheel
        double l = 0.0;
        /// \brief the angle of the right wheel
        double r = 0.0;
    };

    /// \brief The wheel velocities
    struct WheelVelocity {
        /// \brief the velocity of the left wheel
        double l = 0.0;
        /// \brief the velocity of the right wheel
        double r = 0.0;
    };

    /// \brief The configuration of the robot
    struct Config {
        /// \brief the x position of the robot
        double x = 0.0;
        /// \brief the y position of the robot
        double y = 0.0;
        /// \brief the angle of the robot
        double theta = 0.0;
    };

    /// \brief models the kinematics of a differential robot
    class DiffDrive {
        /// \brief the wheel radius
        double w_radius;
        /// \brief the distance between the wheels
        double t_width;
        /// \brief the wheel angle
        WheelAngle phi;
        /// \brief the wheel velocity
        WheelVelocity phi_dot;
        /// \brief the configuration of the robot
        Config q;

        public:
            /// \brief create a DiffDrive object
            DiffDrive();

            /// \brief create a DiffDrive object with the configuration
            /// \param radius - the wheel radius
            /// \param width - the distance between the wheels
            explicit DiffDrive(double radius, double width);

            /// \brief create a DiffDrive object with the configuration, wheel angles,
            /// and wheel velocities
            /// \param radius - the wheel radius
            /// \param width - the distance between the wheels
            /// \param config - the configuration of the robot
            DiffDrive(double radius, double width, Config config);

            /// \brief compute the wheel velocities required to make the robot
            /// move at a given twist
            /// \param twist - the given twist
            /// \return the wheel velocities
            WheelVelocity inverse_kinematics(Twist2D twist);

            /// \brief calculate the twist from wheel velocities
            /// \param vel - the wheel velocities
            /// \return the twist
            Twist2D get_twist(WheelVelocity vel);

            /// \brief update the configuration of the robot
            /// \param angle - the new wheel angles
            /// \return the updated configuration
            void forward_kinematics(WheelAngle angle);

            /// \brief the configuration of the robot
            /// \return the x, y, and theta of the configuration
            Config configuration() const;
    };
}

#endif