#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include <sstream>

TEST_CASE("Inverse", "[transform]") { // Katie, Hughes
    float my_x = 0.;
    float my_y = 1.;
    float my_ang = turtlelib::PI/2;
    turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
    turtlelib::Transform2D Ttest_inv = Ttest.inv();
    REQUIRE((Ttest.inv()).rotation() == -my_ang);
    REQUIRE(turtlelib::almost_equal(Ttest_inv.translation().x, -1.0, 1.0e-5));
    REQUIRE(turtlelib::almost_equal(Ttest_inv.translation().y,  0.0, 1.0e-5));
}

TEST_CASE("Rotation", "[transform]") { // Rintaroh, Shima
    double theta = turtlelib::PI/4;
    turtlelib::Transform2D trans = turtlelib::Transform2D(theta);
    REQUIRE(trans.rotation() == theta);
}

TEST_CASE("Translation", "[transform]") { // Rintaroh, Shima
    turtlelib::Vector2D v;
    v.x = 5.5;
    v.y = 7.2;
    turtlelib::Transform2D trans = turtlelib::Transform2D(v);
    REQUIRE(trans.translation().x == v.x);
    REQUIRE(trans.translation().y == v.y);
}

TEST_CASE("Vector2D Operator()", "[transform]") { // Rintaroh, Shima
    turtlelib::Vector2D v;
    v.x = 5;
    v.y = 6;
    double theta = turtlelib::PI/4;
    turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
    turtlelib::Vector2D vec = {3, 4};
    turtlelib::Vector2D vector = trans(vec);
    REQUIRE(turtlelib::almost_equal(vector.x, 4.29289, 1.0e-4));
    REQUIRE(turtlelib::almost_equal(vector.y, 10.9497, 1.0e-4));
}

TEST_CASE("Twist2D Operator()", "[transform]") { // Rintaroh, Shima
    turtlelib::Vector2D v;
    v.x = 5;
    v.y = 6;
    double theta = turtlelib::PI/4;
    turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
    turtlelib::Twist2D t = {3, 4, 5};
    turtlelib::Twist2D twist = trans(t);
    REQUIRE(turtlelib::almost_equal(twist.w, 3));
    REQUIRE(turtlelib::almost_equal(twist.x, 17.2929, 1.0e-5));
    REQUIRE(turtlelib::almost_equal(twist.y, -8.63604, 1.0e-5));
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
   REQUIRE(turtlelib::almost_equal(tf.rotation(), turtlelib::deg2rad(90), 0.00001));
   REQUIRE(turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001));
   REQUIRE(turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001));
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
   REQUIRE(turtlelib::almost_equal((T_ab_1*=T_bc).translation().x, 4.0));
   REQUIRE(turtlelib::almost_equal((T_ab_2*=T_bc).translation().y, 6.0));
   REQUIRE(turtlelib::almost_equal((T_ab_3*=T_bc).rotation(), (turtlelib::PI/2)));
}