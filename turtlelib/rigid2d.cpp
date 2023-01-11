#include <iostream>
#include "rigid2d.hpp"

std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
    os << "[" << v.x << " " << v.y << "]";
    return os;
}