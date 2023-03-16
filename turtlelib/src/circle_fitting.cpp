#include <iostream>
#include <limits.h>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle_fitting.hpp"
#include <armadillo>

namespace turtlelib {
    Circle fit_circle(std::vector<turtlelib::Vector2D> cluster) {
        double x_sum = 0.0;
        double y_sum = 0.0;
        double num = 0.0;
        
        for (size_t i = 0; i < cluster.size(); i++) {
            x_sum += cluster.at(i).x;
            y_sum += cluster.at(i).y;
            num++;
        }

        double x_hat = x_sum / num;
        double y_hat = y_sum / num;
        double xi = 0.0;
        double yi = 0.0;
        double zi = 0.0;
        double z_sum = 0.0;
        double z_mean = 0.0;
        arma::mat Z{cluster.size(), 4, arma::fill::zeros};

        for (size_t i = 0; i < cluster.size(); i++) {
            xi = cluster.at(i).x - x_hat;
            yi = cluster.at(i).y - y_hat;
            zi = std::pow(xi, 2) + std::pow(yi, 2);
            z_sum += zi;
            Z(i, 0) = zi;
            Z(i, 1) = xi;
            Z(i, 2) = yi;
            Z(i, 3) = 1.0;
        }

        z_mean = z_sum / cluster.size();

        arma::mat M{cluster.size(), 4, arma::fill::zeros};
        M = (Z.t() * Z) / cluster.size();

        arma::mat H{4, 4, arma::fill::zeros};
        H(0, 0) = 8.0 * z_mean;
        H(0, 3) = 2.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;
        H(3, 0) = 2.0;

        arma::mat H_inv = H.i();
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd(U, s, V, Z);

        arma::vec A;
        if (s(3) > std::pow(10, -12)) {
            arma::mat Y = V * arma::diagmat(s) * V.t();
            arma::mat Q = Y * H_inv * Y;

            arma::cx_vec eigval;
            arma::cx_mat eigvec;
            arma::eig_gen(eigval, eigvec, Q);
            arma::vec eigval_real_comp = arma::real(eigval);
            arma::mat eigvec_real_comp = arma::real(eigvec);

            size_t index = 0;
            double eigenval = 1000.0;

            for (size_t i = 0; i < eigval_real_comp.size(); i++) {
                if (eigval_real_comp(i) < eigenval && eigval_real_comp(i) > 0.0) {
                    eigenval = eigval_real_comp(i);
                    index = i;
                }
            }

            arma::vec A_star = eigvec_real_comp.col(index);
            A = arma::solve(Y, A_star);
        }
        else {
            A(0) = V(0, 3);
            A(1) = V(1, 3);
            A(2) = V(2, 3);
            A(3) = V(3, 3);
        }

        double a = -A(1) / (2 * A(0));
        double b = -A(2) / (2 * A(0));
        double R = std::sqrt((std::pow(A(1), 2) + std::pow(A(2), 2) - 4.0 * A(0) * A(3)) / (4.0 *
            std::pow(A(0), 2)));

        turtlelib::Circle circle;
        circle.x = a + x_hat;
        circle.y = b + y_hat;
        circle.r = R;

        return circle;
    }
}