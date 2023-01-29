#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include <sstream>

TEST_CASE("Inverse", "[transform]") { // Katie, Hughes
    float my_x = 0.;
    float my_y = 1.;
    float my_ang = turtlelib::PI/2;
    turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
    turtlelib::Transform2D Ttest_inv = Ttest.inv();
    REQUIRE_THAT((Ttest.inv()).rotation(), Catch::Matchers::WithinAbs(-my_ang, 1.0e-5));
    REQUIRE_THAT(Ttest_inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1.0e-5));
    REQUIRE_THAT(Ttest_inv.translation().y,  Catch::Matchers::WithinAbs(0.0, 1.0e-5));
}

TEST_CASE("Rotation", "[transform]") { // Rintaroh, Shima
    double theta = turtlelib::PI/4;
    turtlelib::Transform2D trans = turtlelib::Transform2D(theta);
    REQUIRE_THAT(trans.rotation(), Catch::Matchers::WithinAbs(theta, 1.0e-5));
}

TEST_CASE("Translation", "[transform]") { // Rintaroh, Shima
    turtlelib::Vector2D v;
    v.x = 5.5;
    v.y = 7.2;
    turtlelib::Transform2D trans = turtlelib::Transform2D(v);
    REQUIRE_THAT(trans.translation().x, Catch::Matchers::WithinAbs(v.x, 1.0e-5));
    REQUIRE_THAT(trans.translation().y, Catch::Matchers::WithinAbs(v.y, 1.0e-5));
}

TEST_CASE("Vector2D Operator()", "[transform]") { // Rintaroh, Shima
    turtlelib::Vector2D v;
    v.x = 5;
    v.y = 6;
    double theta = turtlelib::PI/4;
    turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
    turtlelib::Vector2D vec = {3, 4};
    turtlelib::Vector2D vector = trans(vec);
    REQUIRE_THAT(vector.x, Catch::Matchers::WithinAbs(4.29289, 1.0e-4));
    REQUIRE_THAT(vector.y, Catch::Matchers::WithinAbs(10.9497, 1.0e-4));
}

TEST_CASE("Twist2D Operator()", "[transform]") { // Rintaroh, Shima
    turtlelib::Vector2D v;
    v.x = 5;
    v.y = 6;
    double theta = turtlelib::PI/4;
    turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
    turtlelib::Twist2D t = {3, 4, 5};
    turtlelib::Twist2D twist = trans(t);
    REQUIRE_THAT(twist.w, Catch::Matchers::WithinAbs(3, 1.0e-5));
    REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(17.2929, 1.0e-5));
    REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-8.63604, 1.0e-5));
}

TEST_CASE("Stream insertion operator <<", "[transform]") { // Ava, Zahedi
    turtlelib::Vector2D vec;
    vec.x = 1.0;
    vec.y = 3.4;
    double phi = 0.0;
    turtlelib::Transform2D tf = turtlelib::Transform2D(vec, phi);
    std::string str = "deg: 0 x: 1 y: 3.4";
    std::stringstream sstr;
    sstr << tf;
    REQUIRE(sstr.str() == str);
}

TEST_CASE("Stream extraction operator >>", "[transform]") { // Ava, Zahedi
    turtlelib::Transform2D tf = turtlelib::Transform2D();
    std::stringstream sstr;
    sstr << "deg: 90 x: 1 y: 3.4";
    sstr >> tf;
    REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(turtlelib::deg2rad(90), 0.00001));
    REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(3.4, 0.00001));
}

TEST_CASE("operator *=", "[transform]") { // Megan, Sindelar
    turtlelib::Vector2D trans_ab = {1,2};
    double rotate_ab = 0;
    turtlelib::Transform2D T_ab_1 = {trans_ab, rotate_ab};
    turtlelib::Transform2D T_ab_2 = {trans_ab, rotate_ab};
    turtlelib::Transform2D T_ab_3 = {trans_ab, rotate_ab};
    turtlelib::Vector2D trans_bc = {3,4};
    double rotate_bc = turtlelib::PI/2;
    turtlelib::Transform2D T_bc = {trans_bc, rotate_bc};
    REQUIRE_THAT((T_ab_1*=T_bc).translation().x, Catch::Matchers::WithinAbs(4.0, 1.0e-5));
    REQUIRE_THAT((T_ab_2*=T_bc).translation().y, Catch::Matchers::WithinAbs(6.0, 1.0e-5));
    REQUIRE_THAT((T_ab_3*=T_bc).rotation(), Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-5));
}

TEST_CASE("Normalize Angle", "[rigid2d]") {
    REQUIRE_THAT(turtlelib::normalize_angle(turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1.0e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI / 4), Catch::Matchers::WithinAbs(-turtlelib::PI / 4, 1.0e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(3 * turtlelib::PI / 2), Catch::Matchers::WithinAbs(-turtlelib::PI / 2, 1.0e-5));
    REQUIRE_THAT(turtlelib::normalize_angle(-5 * turtlelib::PI / 2), Catch::Matchers::WithinAbs(-turtlelib::PI / 2, 1.0e-5));
}