#include <iostream>
#include <cmath>
#include <limits.h>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        char c = is.peek();

        if(c == '[') {
            is.get();
            is >> v.x;
            is >> v.y;
        }
        else {
            is >> v.x >> v.y;
        }
        std::cin.clear();
        std::cin.ignore(INT_MAX, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & t) {
        os << "[" << t.w << " " << t.x << " " << t.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & t) {
        char c = is.peek();

        if(c == '[') {
            is.get();
            is >> t.w >> t.x >> t.y;
        }
        else {
            is >> t.w >> t.x >> t.y;
        }
        std::cin.clear();
        std::cin.ignore(INT_MAX, '\n');
        return is;
    }

    Transform2D::Transform2D()
    : tran{0.0, 0.0},
      theta(0.0)
    {}

    Transform2D::Transform2D(Vector2D trans)
    : tran(trans),
      theta(0.0)
    {}

    Transform2D::Transform2D(double radians)
    : tran{0.0, 0.0},
      theta(radians)
    {}

    Transform2D::Transform2D(Vector2D trans, double radians)
    : tran(trans),
      theta(radians)
    {}

    Vector2D Transform2D::operator()(Vector2D v) const {
        Vector2D vec;
        vec.x = v.x * cos(theta) - v.y * sin(theta) + tran.x;
        vec.y = v.x * sin(theta) + v.y * cos(theta) + tran.y;
        return vec;
    }

    Transform2D Transform2D::inv() const {
        Transform2D inv_trans;
        inv_trans.tran.x = -tran.x * cos(theta) - tran.y * sin(theta);
        inv_trans.tran.y = -tran.y * cos(theta) + tran.x * sin(theta);
        inv_trans.theta = -theta;
        return inv_trans;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        tran.x = rhs.tran.x * cos(theta) - rhs.tran.y * sin(theta) + tran.x;
        tran.y = rhs.tran.x * sin(theta) + rhs.tran.y * cos(theta) + tran.y;
        theta = rhs.theta + theta;
        return *this;
    }

    Vector2D Transform2D::translation() const {
        return tran;
    }

    double Transform2D::rotation() const {
        return theta;
    }

    Twist2D Transform2D::operator()(Twist2D t) const {
        Twist2D twist;
        twist.w = t.w;
        twist.x = tran.y * t.w + cos(theta) * t.x - sin(theta) * t.y;
        twist.y = -tran.x * t.w + sin(theta) * t.x + cos(theta) * t.y;
        return twist;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        os << "deg: " << rad2deg(tf.theta) << " x: " << tf.tran.x << " y: " << tf.tran.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        Vector2D vec;
        double theta = 0.0;
        char c = is.peek();
        std::string str1;
        std::string str2;
        std::string str3;

        if(c == 'd') {
            is >> str1 >> theta >> str2 >> vec.x >> str3 >> vec.y;
        }
        else {
            is >> theta >> vec.x >> vec.y;
        }
        std::cin.clear();
        std::cin.ignore(INT_MAX, '\n');
        theta = deg2rad(theta);
        tf = Transform2D(vec, theta);
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        return lhs *= rhs;
    }

    Vector2D normalize_vector(Vector2D vec) {
        Vector2D unit_vec;
        double mag = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
        unit_vec.x = vec.x / mag;
        unit_vec.y = vec.y / mag;
        return unit_vec;
    }
}