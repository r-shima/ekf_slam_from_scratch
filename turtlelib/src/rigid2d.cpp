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
        auto c = is.peek();

        if(c == '[') {
            is.get();
            is >> v.x;
            is >> v.y;
        }
        else {
            is >> v.x >> v.y;
        }
        is.ignore(INT_MAX, '\n');
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
        is.ignore(INT_MAX, '\n');
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
        return {v.x * cos(theta) - v.y * sin(theta) + tran.x, v.x * sin(theta) + v.y * cos(theta) + tran.y};
    }

    Transform2D Transform2D::inv() const {
        return {{-tran.x * cos(theta) - tran.y * sin(theta), -tran.y * cos(theta) + tran.x * sin(theta)}, -theta};
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
        return {t.w, tran.y * t.w + cos(theta) * t.x - sin(theta) * t.y, -tran.x * t.w + sin(theta) * t.x + cos(theta) * t.y};
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
        is.clear();
        is.ignore(INT_MAX, '\n');
        theta = deg2rad(theta);
        tf = Transform2D(vec, theta);
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        return lhs *= rhs;
    }

    Vector2D normalize_vector(Vector2D vec) {
        const auto mag = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
        return {vec.x / mag, vec.y / mag};
    }

    double normalize_angle(double rad) {
        // double angle = fmod(rad, 2*PI);
        // if (angle > PI) {
        //     angle = -PI + (angle - PI);
        // }
        // else if(angle <= -PI) {
        //     angle = PI - (angle + PI);
        // }
        // return angle;

        // double deg = rad2deg(rad);
        // return deg2rad(remainder(deg, 360.0));

        rad = fmod(rad, 2 * PI);
        if (rad < -PI) {
            rad += 2 * PI;
        } 
        else if (rad > PI) {
            rad -= 2 * PI;
        }
        return rad;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & rhs) {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs) {
        return lhs += rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs) {
        return lhs -= rhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs) {
        return lhs *= rhs;
    }

    Vector2D operator*(const double & lhs, Vector2D rhs) {
        return rhs *= lhs;
    }

    double dot(Vector2D vec1, Vector2D vec2) {
        double dot_product = vec1.x * vec2.x + vec1.y * vec2.y;
        return dot_product;
    }

    double magnitude(Vector2D vec) {
        double mag = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
        return mag;
    }

    double angle(Vector2D vec1, Vector2D vec2) {
        double dot_product = dot(vec1, vec2);
        double mag1 = magnitude(vec1);
        double mag2 = magnitude(vec2);
        double angle = acos(dot_product / (mag1 * mag2));
        return angle;
    }

    Transform2D integrate_twist(Twist2D twist) {
        Transform2D T_bbp, T_sb, T_ssp, T_bs;
        Vector2D vec;
        if(twist.w == 0.0) {
            vec.x = twist.x;
            vec.y = twist.y;
            T_bbp = Transform2D(vec);
            return T_bbp;
        }
        else {
            vec.x = twist.y / twist.w;
            vec.y = -twist.x / twist.w;
            T_sb = Transform2D(vec);
            T_ssp = Transform2D(twist.w);
            T_bs = T_sb.inv();
            T_bbp = T_bs * T_ssp * T_sb;
            return T_bbp;
        }
    }
}