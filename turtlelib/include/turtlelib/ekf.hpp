#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP

#include <iosfwd>
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <unordered_set>

namespace turtlelib {
    /// \brief the number of landmarks
    constexpr int n = 3;

    /// \brief implements the Extended Kalman Filter
    class EKF {
        /// \brief the combined state vector containing the robot state and the map state
        arma::mat xi;
        /// \brief the covariance
        // arma::mat covariance;
        /// \brief the covariance matrix used for the process noise
        arma::mat Q{arma::mat(3, 3, arma::fill::eye)};
        /// \brief the estimated combined state vector
        arma::mat estimated_xi{arma::colvec(2*n+3, arma::fill::zeros)};
        /// \brief the estimated covariance
        // arma::mat estimated_covariance{arma::mat(2*n+3, 2*n+3, arma::fill::zeros)};
        arma::mat estimated_covariance;
        /// \brief the change in twist
        arma::colvec ut{arma::colvec(2*n+3, 1, arma::fill::zeros)};
        /// \brief the previous twist
        Twist2D prev_twist{0.0, 0.0, 0.0};
        /// \brief the covariance matrix used for the sensor noise
        arma::mat R{arma::mat(2, 2, arma::fill::eye)};
        /// \brief the landmarks that were seen
        std::unordered_set<double> landmarks;

        public:
            /// \brief create an EKF object
            EKF();

            /// \brief create an EKF object with the configuration
            /// \param config - the robot configuration
            EKF(Config config);

            /// \brief initialize the robot state in the combined state vector
            /// \param config - the robot configuration
            /// \return none
            void initialize_state(Config config);

            /// \brief initialize the covariance
            /// \return none
            void initialize_covariance();

            /// \brief compute the At matrix
            /// \param twist - the twist
            /// \return the At matrix
            arma::mat calculate_At(arma::colvec twist);

            /// \brief predict the state and uncertainty
            /// \param twist - the twist
            /// \return none
            void predict(Twist2D twist);

            /// \brief update the state and uncertainty
            /// \param x - the x coordinate of the landmark
            /// \param y - the y coordinate of the landmark
            /// \param j - the marker id
            /// \return none
            void update(double x, double y, size_t j);

            /// \brief get the robot configuration
            /// \return the x, y, and theta of the configuration
            Config get_configuration();

            /// \brief get the predicted robot configuration
            // \return the predicted x, y, and theta of the configuration
            Config get_predicted_configuration();

            arma::mat covariance;
    };
}

#endif