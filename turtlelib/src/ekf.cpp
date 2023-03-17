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

    void EKF::predict(Twist2D twist) {
        ut(0) = normalize_angle(twist.w - prev_twist.w);
        ut(1) = twist.x - prev_twist.x;
        ut(2) = twist.y - prev_twist.y;
        prev_twist = twist;
        estimated_xi(0) = estimated_xi(0) + ut(0);
        estimated_xi(1) = estimated_xi(1) + ut(1);
        estimated_xi(2) = estimated_xi(2) + ut(2);

        arma::mat zero_mat1, zero_mat2, zero_mat3, Q_bar;
        zero_mat1 = arma::mat(3, 2*n, arma::fill::zeros);
        zero_mat2 = arma::mat(2*n, 3, arma::fill::zeros);
        zero_mat3 = arma::mat(2*n, 2*n, arma::fill::zeros);
        arma::mat Q{arma::mat(3, 3, arma::fill::eye)};
        double Q_noise = 0.01;
        Q *= Q_noise;
        Q_bar = arma::join_rows(arma::join_cols(Q, zero_mat2),
            arma::join_cols(zero_mat1, zero_mat3));
        arma::mat At{2*n+3, 2*n+3, arma::fill::eye};
        At(1, 0) = -ut(2);
        At(2, 0) = ut(1);
        estimated_covariance = At * covariance * At.t() + Q_bar;
    }

    void EKF::update(double x, double y, size_t j) {
        double r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        double phi = atan2(y, x);
        arma::colvec z{r, phi};

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
        arma::colvec z_hat{r_hat, phi_hat};

        arma::mat mat1(2, 3);
        arma::mat mat2(2, 2*j, arma::fill::zeros);
        arma::mat mat3(2, 2);
        arma::mat mat4(2, 2*n-2*(j+1), arma::fill::zeros);

        mat1(0, 0) = 0.0;
        mat1(0, 1) = -delta_x / std::sqrt(d);
        mat1(0, 2) = -delta_y / std::sqrt(d);
        mat1(1, 0) = -1.0;
        mat1(1, 1) = delta_y / d;
        mat1(1, 2) = -delta_x / d;

        mat3(0, 0) = delta_x / std::sqrt(d);
        mat3(0, 1) = delta_y / std::sqrt(d);
        mat3(1, 0) = -delta_y / d;
        mat3(1, 1) = delta_x / d;

        arma::mat H = arma::join_rows(mat1, mat2, mat3, mat4);
        arma::mat R{arma::mat(2, 2, arma::fill::eye)};
        double R_noise = 0.01;
        R *= R_noise;
        arma::mat K = estimated_covariance * H.t() * ((H * estimated_covariance * H.t() + R)).i();
        arma::colvec z_diff = z - z_hat;
        z_diff(1) = normalize_angle(z_diff(1));
        xi = estimated_xi + K * z_diff;
        xi(0) = normalize_angle(xi(0));
        estimated_xi = xi;
        arma::mat I{2*n+3, 2*n+3, arma::fill::eye};
        covariance = (I - K * H) * estimated_covariance;
        estimated_covariance = covariance;
    }

    size_t EKF::data_association(double x, double y) {
        double r = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        double phi = atan2(y, x);
        arma::colvec z{r, phi};

        arma::colvec temp = estimated_xi;
        temp(2*N+3+1) = temp(1) + r * cos(phi + temp(0));
        temp(2*N+4+1) = temp(2) + r * sin(phi + temp(0));
        std::vector<double> distance_list;

        for (int i = 0; i < N+1; i++) {
            double delta_x = temp(2*i+3) - temp(1);
            double delta_y = temp(2*i+4) - temp(2);
            double d = std::pow(delta_x, 2) + std::pow(delta_y, 2);
            double r_hat = std::sqrt(d);
            double phi_hat = normalize_angle(atan2(delta_y, delta_x) - temp(0));
            arma::colvec z_hat{r_hat, phi_hat};

            arma::mat mat1(2, 3);
            arma::mat mat2(2, 2*i, arma::fill::zeros);
            arma::mat mat3(2, 2);
            arma::mat mat4(2, 2*n-2*(i+1), arma::fill::zeros);

            mat1(0, 0) = 0.0;
            mat1(0, 1) = -delta_x / std::sqrt(d);
            mat1(0, 2) = -delta_y / std::sqrt(d);
            mat1(1, 0) = -1.0;
            mat1(1, 1) = delta_y / d;
            mat1(1, 2) = -delta_x / d;

            mat3(0, 0) = delta_x / std::sqrt(d);
            mat3(0, 1) = delta_y / std::sqrt(d);
            mat3(1, 0) = -delta_y / d;
            mat3(1, 1) = delta_x / d;

            arma::mat H = arma::join_rows(mat1, mat2, mat3, mat4);
            arma::mat R{arma::mat(2, 2, arma::fill::eye)};
            double R_noise = 0.01;
            R *= R_noise;
            arma::mat cov_k = H * estimated_covariance * H.t() + R;
            arma::colvec z_diff = z - z_hat;
            z_diff(1) = normalize_angle(z_diff(1));
            arma::mat d_k = z_diff.t()  * cov_k.i() * z_diff;
            distance_list.push_back(d_k(0));
        }

        bool is_new = true;
        size_t min_index = N + 1;
        double threshold = distance_list.at(distance_list.size() - 1);

        for (size_t i = 0; i < distance_list.size(); i++) {
            if (distance_list.at(i) < threshold) {
                threshold = distance_list.at(i);
                min_index = i;
                is_new = false;
            }
        }

        if (is_new == true) {
            N++;
        }

        return min_index;
    }

    Config EKF::get_configuration() {
        return {xi(1), xi(2), xi(0)};
    }

    arma::mat EKF::get_xi() {
        return xi;
    }
}