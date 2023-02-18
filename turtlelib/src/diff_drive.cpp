#include <iostream>
#include <limits.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"

namespace turtlelib {
    DiffDrive::DiffDrive()
    : w_radius{wheel_radius},
      t_width{track_width},
      phi{0.0, 0.0},
      q{0.0, 0.0, 0.0}
    {}

    DiffDrive::DiffDrive(double radius, double width)
    : w_radius{radius},
      t_width{width},
      phi{0.0, 0.0},
      q{0.0, 0.0, 0.0}
    {}

    DiffDrive::DiffDrive(double radius, double width, Config config)
    : w_radius{radius},
      t_width{width},
      phi{0.0, 0.0},
      q{config.x, config.y, config.theta}
    {}

    WheelVelocity DiffDrive::inverse_kinematics(Twist2D twist) {
        if (twist.y != 0.0) {
            throw std::logic_error("The wheels should not be slipping");
        }
        // Refer to equations (1) and (2) in Kinematics.pdf
        phi_dot.l = (1 / w_radius) * (-(t_width / 2) * twist.w + twist.x);
        phi_dot.r = (1 / w_radius) * ((t_width / 2) * twist.w + twist.x);
        return phi_dot;
    }

    Twist2D DiffDrive::get_twist(WheelVelocity vel) {
        Twist2D twist;
        // Refer to equations (3) and (4) in Kinematics.pdf
        twist.w = (w_radius / 2) * (-vel.l / (t_width / 2) + vel.r / (t_width / 2));
        twist.x = (w_radius / 2) * (vel.l + vel.r);
        twist.y = 0.0;
        return twist;
    }

    void DiffDrive::forward_kinematics(WheelAngle angle) {
        phi_dot.l = angle.l - phi.l;
        phi_dot.r = angle.r - phi.r;
        Twist2D twist = get_twist(phi_dot);
        
        Vector2D vec;
        vec.x = q.x;
        vec.y = q.y;
        double theta = q.theta;
        Transform2D T_wb(vec, theta);
        Transform2D T_bbp = integrate_twist(twist);
        Transform2D T_wbp = T_wb * T_bbp;

        q.x = T_wbp.translation().x;
        q.y = T_wbp.translation().y;
        q.theta = normalize_angle(T_wbp.rotation());

        // This also works (refer to equation (5) in Kinematics.pdf)
        // q.x = q.x + T_bbp.translation().x * cos(q.theta) - T_bbp.translation().y * sin(q.theta);
        // q.y = q.y + T_bbp.translation().x * sin(q.theta) + T_bbp.translation().y * cos(q.theta);
        // q.theta = normalize_angle(T_bbp.rotation());
    }

    Config DiffDrive::configuration() const {
        return q;
    }

    Twist2D DiffDrive::twist_to_angles(WheelAngle angle) {
        phi_dot.l = angle.l - phi.l;
        phi_dot.r = angle.r - phi.r;
        Twist2D twist = get_twist(phi_dot);
        return twist;
    }

    void DiffDrive::set_configuration(Config config) {
        q.x = config.x;
        q.y = config.y;
        q.theta = config.theta;
    }
}