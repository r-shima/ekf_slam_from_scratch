#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle_fitting.hpp"
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

TEST_CASE("Vector2D Operator+", "[rigid2d]") {
    turtlelib::Vector2D vec1, vec2;
    vec1.x = 2.5;
    vec1.y = 4.3;
    vec2.x = 3.0;
    vec2.y = 5.2;
    REQUIRE_THAT((vec1+vec2).x, Catch::Matchers::WithinAbs(5.5, 1.0e-5));
    REQUIRE_THAT((vec1+vec2).y, Catch::Matchers::WithinAbs(9.5, 1.0e-5));
}

TEST_CASE("Vector2D Operator-", "[rigid2d]") {
    turtlelib::Vector2D vec1, vec2;
    vec1.x = 2.5;
    vec1.y = 4.3;
    vec2.x = 3.0;
    vec2.y = 5.2;
    REQUIRE_THAT((vec1-vec2).x, Catch::Matchers::WithinAbs(-0.5, 1.0e-5));
    REQUIRE_THAT((vec1-vec2).y, Catch::Matchers::WithinAbs(-0.9, 1.0e-5));
}

TEST_CASE("Operator* Vector2D and Scalar", "[rigid2d]") {
    turtlelib::Vector2D vec = {2.0, 3.0};
    double scalar = 3.0;
    REQUIRE_THAT((vec*scalar).x, Catch::Matchers::WithinAbs(6.0, 1.0e-5));
    REQUIRE_THAT((vec*scalar).y, Catch::Matchers::WithinAbs(9.0, 1.0e-5));
}

TEST_CASE("Operator* Scalar and Vector2D", "[rigid2d]") {
    turtlelib::Vector2D vec = {2.0, 3.0};
    double scalar = 3.0;
    REQUIRE_THAT((scalar*vec).x, Catch::Matchers::WithinAbs(6.0, 1.0e-5));
    REQUIRE_THAT((scalar*vec).y, Catch::Matchers::WithinAbs(9.0, 1.0e-5));
}

TEST_CASE("Dot Product", "[rigid2d]") {
    turtlelib::Vector2D vec1, vec2;
    vec1.x = 1.0;
    vec1.y = 2.0;
    vec2.x = 1.0;
    vec2.y = 2.0;
    REQUIRE_THAT(turtlelib::dot(vec1, vec2), Catch::Matchers::WithinAbs(5.0, 1.0e-5));
}

TEST_CASE("Magnitude", "[rigid2d]") {
    turtlelib::Vector2D vec;
    vec.x = 3.0;
    vec.y = 4.0;
    REQUIRE_THAT(turtlelib::magnitude(vec), Catch::Matchers::WithinAbs(5.0, 1.0e-5));
}

TEST_CASE("Angle", "[rigid2d]") {
    turtlelib::Vector2D vec1, vec2;
    vec1.x = 3.0;
    vec1.y = 4.0;
    vec2.x = 4.0;
    vec2.y = 3.0;
    REQUIRE_THAT(turtlelib::angle(vec1, vec2), Catch::Matchers::WithinAbs(0.28379, 1.0e-5));
}

TEST_CASE("Integrate Twist, Pure Translation", "[rigid2d]") {
    turtlelib::Twist2D twist;
    twist.w = 0.0;
    twist.x = 5.0;
    twist.y = 6.0;
    turtlelib::Transform2D tf = integrate_twist(twist);
    turtlelib::Vector2D tran = tf.translation();
    REQUIRE_THAT(tran.x, Catch::Matchers::WithinAbs(5.0, 1.0e-5));
    REQUIRE_THAT(tran.y, Catch::Matchers::WithinAbs(6.0, 1.0e-5));
}

TEST_CASE("Integrate Twist, Pure Rotation", "[rigid2d]") {
    turtlelib::Twist2D twist;
    twist.w = turtlelib::PI / 4;
    twist.x = 0.0;
    twist.y = 0.0;
    turtlelib::Transform2D tf = integrate_twist(twist);
    double theta = tf.rotation();
    REQUIRE_THAT(theta, Catch::Matchers::WithinAbs(0.785398, 1.0e-5));
}

TEST_CASE("Integrate Twist, Simultaneous Translation and Rotation", "[rigid2d]") {
    turtlelib::Twist2D twist;
    twist.w = turtlelib::PI / 4;
    twist.x = 5.0;
    twist.y = 6.0;
    turtlelib::Transform2D tf = integrate_twist(twist);
    turtlelib::Vector2D tran = tf.translation();
    double theta = tf.rotation();
    REQUIRE_THAT(tran.x, Catch::Matchers::WithinAbs(2.26404, 1.0e-5));
    REQUIRE_THAT(tran.y, Catch::Matchers::WithinAbs(7.26651, 1.0e-5));
    REQUIRE_THAT(theta, Catch::Matchers::WithinAbs(0.785398, 1.0e-5));
}

TEST_CASE("Inverse Kinematics, Driving Forward", "[diff_drive]") {
    turtlelib::Twist2D twist;
    twist.w = 0.0;
    twist.x = 1.0;
    twist.y = 0.0;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelVelocity vel = d.inverse_kinematics(twist);
    REQUIRE_THAT(vel.l, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(vel.r, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
}

TEST_CASE("Inverse Kinematics, Pure Rotation", "[diff_drive]") {
    turtlelib::Twist2D twist;
    twist.w = turtlelib::PI;
    twist.x = 0.0;
    twist.y = 0.0;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelVelocity vel = d.inverse_kinematics(twist);
    REQUIRE_THAT(vel.l, Catch::Matchers::WithinAbs(-6.2831853072, 1.0e-5));
    REQUIRE_THAT(vel.r, Catch::Matchers::WithinAbs(6.2831853072, 1.0e-5));
}

TEST_CASE("Inverse Kinematics, Following Arc of a Circle", "[diff_drive]") {
    turtlelib::Twist2D twist;
    twist.w = turtlelib::PI;
    twist.x = 1.0;
    twist.y = 0.0;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelVelocity vel = d.inverse_kinematics(twist);
    REQUIRE_THAT(vel.l, Catch::Matchers::WithinAbs(-5.2831853072, 1.0e-5));
    REQUIRE_THAT(vel.r, Catch::Matchers::WithinAbs(7.2831853072, 1.0e-5));
}

TEST_CASE("Inverse Kinematics, Impossible-to-follow Twist", "[diff_drive]") {
    turtlelib::Twist2D twist;
    twist.w = turtlelib::PI;
    twist.x = 1.0;
    twist.y = 3.0;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    turtlelib::WheelVelocity vel;
    REQUIRE_THROWS(vel = d.inverse_kinematics(twist));
}

TEST_CASE("Forward Kinematics, Driving Forward", "[diff_drive]") {
    turtlelib::WheelAngle angle;
    angle.l = turtlelib::PI;
    angle.r = turtlelib::PI;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    d.forward_kinematics(angle);
    REQUIRE_THAT(d.configuration().x, Catch::Matchers::WithinAbs(3.1415926536, 1.0e-5));
    REQUIRE_THAT(d.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(d.configuration().theta, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
}

TEST_CASE("Forward Kinematics, Pure Rotation", "[diff_drive]") {
    turtlelib::WheelAngle angle;
    angle.l = turtlelib::PI;
    angle.r = -turtlelib::PI;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    d.forward_kinematics(angle);
    REQUIRE_THAT(d.configuration().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(d.configuration().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(d.configuration().theta, Catch::Matchers::WithinAbs(-1.5707963268, 1.0e-5));
}

TEST_CASE("Forward Kinematics, Following Arc of a Circle", "[diff_drive]") {
    turtlelib::WheelAngle angle;
    angle.l = turtlelib::PI;
    angle.r = turtlelib::PI / 2;
    double radius = 1.0;
    double width = 4.0;
    turtlelib::DiffDrive d = turtlelib::DiffDrive(radius, width);
    d.forward_kinematics(angle);
    REQUIRE_THAT(d.configuration().x, Catch::Matchers::WithinAbs(2.2961005942, 1.0e-5));
    REQUIRE_THAT(d.configuration().y, Catch::Matchers::WithinAbs(-0.4567228049, 1.0e-5));
    REQUIRE_THAT(d.configuration().theta, Catch::Matchers::WithinAbs(-0.3926990817, 1.0e-5));
}

TEST_CASE("Circle Fitting Test 1", "[circle_fitting]") {
    turtlelib::Vector2D a, b, c, d, e, f;
    a.x = 1.0;
    a.y = 7.0;
    b.x = 2.0;
    b.y = 6.0;
    c.x = 5.0;
    c.y = 8.0;
    d.x = 7.0;
    d.y = 7.0;
    e.x = 9.0;
    e.y = 5.0;
    f.x = 3.0;
    f.y = 7.0;
    std::vector<turtlelib::Vector2D> cluster{a, b, c, d, e, f};
    turtlelib::Circle circle = turtlelib::fit_circle(cluster);
    REQUIRE_THAT(circle.x, Catch::Matchers::WithinAbs(4.615482, 1.0e-4));
    REQUIRE_THAT(circle.y, Catch::Matchers::WithinAbs(2.807354, 1.0e-4));
    REQUIRE_THAT(circle.r, Catch::Matchers::WithinAbs(4.8275, 1.0e-4));
}

TEST_CASE("Circle Fitting Test 2", "[circle_fitting]") {
    turtlelib::Vector2D a, b, c, d;
    a.x = -1.0;
    a.y = 0.0;
    b.x = -0.3;
    b.y = -0.06;
    c.x = 0.3;
    c.y = 0.1;
    d.x = 1.0;
    d.y = 0.0;
    std::vector<turtlelib::Vector2D> cluster{a, b, c, d};
    turtlelib::Circle circle = turtlelib::fit_circle(cluster);
    REQUIRE_THAT(circle.x, Catch::Matchers::WithinAbs(0.4908357, 1.0e-4));
    REQUIRE_THAT(circle.y, Catch::Matchers::WithinAbs(-22.15212, 1.0e-4));
    REQUIRE_THAT(circle.r, Catch::Matchers::WithinAbs(22.17979, 1.0e-4));
}