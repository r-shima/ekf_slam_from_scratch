#include <iostream>
#include "rigid2d.hpp"

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
            is.get();
        }
        else {
            is >> v.x >> v.y;
        }
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
            is >> t.w;
            is >> t.x;
            is >> t.y;
            is.get();
        }
        else {
            is >> t.w >> t.x >> t.y;
        }
        return is;
    }
}

int main() {
    // printf("%d\n", turtlelib::almost_equal(3, 2));
    // turtlelib::Vector2D vec1;
    // vec1.x = 1.0;
    // vec1.y = 2.0;
    // std::cout << vec1 << std::endl;
    // turtlelib::Vector2D vec2;
    // std::cin >> vec2;
    // std::cout << vec2;
    // std::cin.ignore(100, '\n');
    // turtlelib::Twist2D vec3;
    // vec3.w = 1.0;
    // vec3.x = 1.0;
    // vec3.y = 1.0;
    // std::cout << vec3 << std::endl;
    // turtlelib::Twist2D vec4;
    // std::cin >> vec4;
    // std::cout << vec4;
    return 0;
}