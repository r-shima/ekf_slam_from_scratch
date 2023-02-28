#include <iostream>
#include <limits.h>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <armadillo>

namespace turtlelib {
    EKF::EKF()
    : xi{arma::colvec(2*n+3, arma::fill::zeros)}
    {
        initialize_covariance();
    }

    EKF::EKF(Config config)
    : xi{arma::colvec(2*n+3, arma::fill::zeros)}
    {
        initialize_state(config);
        initialize_covariance();
    }

    void EKF::initialize_state(Config config) {
        xi(0) = config.theta;
        xi(1) = config.x;
        xi(2) = config.y;
    }

    void EKF::initialize_covariance() {
        arma::mat sigma_0q, zero_mat1, zero_mat2, sigma_0m;
        sigma_0q = arma::mat(3, 3, arma::fill::zeros);
        zero_mat1 = arma::mat(3, 2*n, arma::fill::zeros);
        zero_mat2 = arma::mat(2*n, 3, arma::fill::zeros);
        sigma_0m = arma::mat(2*n, 2*n, arma::fill::eye) * 100000.0;
        covariance = arma::join_rows(arma::join_cols(sigma_0q, zero_mat2),
            arma::join_cols(zero_mat1, sigma_0m));
    }

    arma::mat EKF::calculate_At(arma::colvec twist) {
        double theta = xi(0);
        arma::mat zero_mat1, zero_mat2, zero_mat3, mat1, mat2, I, At;
        zero_mat1 = arma::mat(3, 2*n, arma::fill::zeros);
        zero_mat2 = arma::mat(2*n, 3, arma::fill::zeros);
        zero_mat3 = arma::mat(2*n, 2*n, arma::fill::zeros);
        mat1 = arma::mat(3, 3, arma::fill::zeros);
        I = arma::mat(2*n+3, 2*n+3, arma::fill::eye);

        if (almost_equal(twist(0), 0.0)) {
            mat1(1, 0) = -twist(0) * sin(theta);
            mat1(2, 0) = twist(0) * cos(theta);
        }
        else {
            mat1(1, 0) = -(twist(1) / twist(0)) * cos(theta) + (twist(1) / twist(0)) *
                cos(theta + twist(0));
            mat1(2, 0) = -(twist(1) / twist(0)) * sin(theta) + (twist(1) / twist(0)) *
                sin(theta + twist(0));
        }

        mat2 = arma::join_rows(arma::join_cols(mat1, zero_mat2),
            arma::join_cols(zero_mat1, zero_mat3));
        At = I + mat2;
        return At;
    }

    void EKF::predict(Twist2D twist) {
        // double theta = xi(0, 0);
        ut(0) = normalize_angle(twist.w - prev_twist.w);
        ut(1) = twist.x - prev_twist.x;
        ut(2) = twist.y - prev_twist.y;
        prev_twist = twist;
        estimated_xi(0) = xi(0) + ut(0);
        estimated_xi(1) = xi(1) + ut(1);
        estimated_xi(2) = xi(2) + ut(2);

        // if (almost_equal(ut.w, 0.0)) {
        //     arma::colvec delta_config{2*n+3, arma::fill::zeros};
        //     // arma::colvec noise{2*n+3, arma::fill::zeros};
        //     delta_config(1, 0) = ut.x * cos(theta);
        //     delta_config(2, 0) = ut.x * sin(theta);
            
        //     // noise(0, 0) = 
        //     // noise(1, 0) = 
        //     // noise(2, 0) = 

        //     estimated_xi += xi + delta_config; // + noise;
        // }
        // else {
        //     arma::colvec delta_config{2*n+3, arma::fill::zeros};
        //     delta_config(0, 0) = ut.w;
        //     delta_config(1, 0) = -(ut.x / ut.w) * sin(theta) + (ut.x / ut.w) *
        //         sin(theta + ut.w);
        //     delta_config(2, 0) = (ut.x / ut.w) * cos(theta) - (ut.x / ut.w) *
        //         cos(theta + ut.w);

        //     estimated_xi += xi + delta_config;
        // }

        arma::mat zero_mat1, zero_mat2, zero_mat3, Q_bar;
        zero_mat1 = arma::mat(3, 2*n, arma::fill::zeros);
        zero_mat2 = arma::mat(2*n, 3, arma::fill::zeros);
        zero_mat3 = arma::mat(2*n, 2*n, arma::fill::zeros);
        Q_bar = arma::join_rows(arma::join_cols(Q, zero_mat2),
            arma::join_cols(zero_mat1, zero_mat3));
        arma::mat At = calculate_At(ut);
        estimated_covariance = At * covariance * At.t() + Q_bar;
    }

    // arma::colvec EKF::calculate_h(int j) {
    //     double landmark_x, landmark_y, xt, yt, thetat, r, phi;
    //     arma::colvec z{2, arma::fill::zeros};

    //     landmark_x = xi(2*j+3, 0);
    //     landmark_y = xi(2*j+4, 0);
    //     thetat = xi(0, 0);
    //     xt = xi(1, 0);
    //     yt = xi(2, 0);
    //     z(0) = std::sqrt(std::pow(landmark_x - xt, 2) + std::pow(landmark_y - yt, 2));
    //     z(1) = normalize_angle(atan2(landmark_y - yt, landmark_x - xt) - thetat);
    //     return z;
    // }

    // arma::mat EKF::calculate_H(int j) {
    //     double delta_x = estimated_xi(2*j+3, 0) - estimated_xi(1, 0);
    //     double delta_y = estimated_xi(2*j+4, 0) - estimated_xi(2, 0);
    //     double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);
    //     double r_hat = std::sqrt(d);

    //     arma::mat mat1, mat2, mat3, mat4, H;
    //     mat1 = arma::mat(2, 3, arma::fill::zeros);
    //     mat1(0, 1) = -delta_x / std::sqrt(d);
    //     mat1(0, 2) = -delta_y / std::sqrt(d);
    //     mat1(1, 0) = -1.0;
    //     mat1(1, 1) = delta_y / d;
    //     mat1(1, 2) = -delta_x / d;

    //     mat2 = arma::mat(2, 2*(j-1), arma::fill::zeros);

    //     mat3 = arma::mat(2, 2, arma::fill::zeros);
    //     mat3(0, 0) = delta_x / std::sqrt(d);
    //     mat3(0, 1) = delta_y / std::sqrt(d);
    //     mat3(1, 0) = -delta_y / d;
    //     mat3(1, 1) = delta_x / d;
        
    //     mat4 = arma::mat(2, 2*n-2*j, arma::fill::zeros);

    //     H = arma::join_rows(mat1, mat2, mat3, mat4);
    //     return H;
    // }

    void EKF::update(double x, double y, size_t j) {
        double r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        double phi = atan2(y, x);
        arma::colvec z{2, arma::fill::zeros};
        z(0) = r;
        z(1) = phi;

        if (landmarks.find(j) == landmarks.end()) {
            estimated_xi(2*j+3) = estimated_xi(1) + r * cos(phi + estimated_xi(0));
            estimated_xi(2*j+4) = estimated_xi(2) + r * sin(phi + estimated_xi(0));
            landmarks.insert(j);
        }

        double delta_x = estimated_xi(2*j+3) - estimated_xi(1);
        double delta_y = estimated_xi(2*j+4) - estimated_xi(2);
        double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);
        double r_hat = std::sqrt(d);
        double phi_hat = normalize_angle(atan2(delta_y, delta_x) - estimated_xi(0));
        arma::colvec z_hat{2, arma::fill::zeros};
        z_hat(0) = r_hat;
        z_hat(1) = phi_hat;

        arma::mat mat1, mat2, mat3, mat4, H;
        mat1 = {{0.0, -delta_x / std::sqrt(d), -delta_y / std::sqrt(d)},
                {-1.0, delta_y / d, -delta_x / d}};
        mat2 = arma::mat(2, 2*j, arma::fill::zeros);
        mat3 = {{delta_x / std::sqrt(d), delta_y / std::sqrt(d)},
                {-delta_y / d, -delta_x / d}};
        mat4 = arma::mat(2, 2*n-2*(j+1), arma::fill::zeros);
        H = arma::join_rows(mat1, mat2, mat3, mat4);
        arma::mat K = estimated_covariance * H.t() * (H * estimated_covariance * H.t() + R).i();
        // Rj = R.submat(j, j, j+1, j+1);
        xi = estimated_xi + K * (z - z_hat);
        // xi(0) = normalize_angle(xi(0));
        arma::mat I{2*n+3, 2*n+3, arma::fill::eye};
        covariance = (I - K * H) * estimated_covariance;
        estimated_covariance = covariance;
    }

    Config EKF::get_configuration() {
        return {xi(1, 0), xi(2, 0), xi(0, 0)};
    }

    Config EKF::get_predicted_configuration() {
        return {estimated_xi(1, 0), estimated_xi(2, 0), estimated_xi(0, 0)};
    }
}