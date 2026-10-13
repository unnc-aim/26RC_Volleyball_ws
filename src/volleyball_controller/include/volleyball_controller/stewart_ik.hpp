//
// Created by txy on 7/8/26.
//

#ifndef STEWART_IK_HPP
#define STEWART_IK_HPP


#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace aim::algorithm::ik::stewart
{
    class StewartIk
    {
    public:
        StewartIk()
        {
            //Base Angle
            phi_b_ = {
                0, 0, 120.0 * M_PI / 180.0, 120.0 * M_PI / 180.0, 240.0 * M_PI / 180.0, 240.0 * M_PI / 180.0
            };

            // Platform Angle
            lambda_p_ = {
                0, 0, 120.0 * M_PI / 180.0, 120.0 * M_PI / 180.0, 240.0 * M_PI / 180.0, 240.0 * M_PI / 180.0
            };

            const double rbi = std::sqrt(rb_ * rb_ + ei_ * ei_);
            const double rpi = std::sqrt(rp_ * rp_ + bi_ * bi_);
            base_ang_offset_ = std::atan(ei_ / rb_);
            plat_ang_offset_ = std::atan(bi_ / rp_);

            for (int i = 0; i < 6; ++i)
            {
                double pair_sign = (i % 2 == 0) ? -1.0 : 1.0;
                double up_ang = lambda_p_[i] + pair_sign * plat_ang_offset_;
                double down_ang = phi_b_[i] + pair_sign * base_ang_offset_;

                pi_local_[i] = Eigen::Vector3d(rpi * std::cos(up_ang), rpi * std::sin(up_ang), zrp_);
                B_[i] = Eigen::Vector3d(rbi * std::cos(down_ang), rbi * std::sin(down_ang), 0.0);
            }

            normals_[0] = Eigen::Vector3d(1.0, 0.0, 0.0);
            Dvals_[0] = -normals_[0].dot(B_[0]);

            Eigen::Vector3d v34 = B_[3] - B_[2];
            normals_[1] = Eigen::Vector3d(-v34.y(), v34.x(), 0.0).normalized();
            Dvals_[1] = -normals_[1].dot(B_[2]);

            Eigen::Vector3d v56 = B_[5] - B_[4];
            normals_[2] = Eigen::Vector3d(-v56.y(), v56.x(), 0.0).normalized();
            Dvals_[2] = -normals_[2].dot(B_[4]);

            for (int i = 0; i < 6; ++i)
            {
                outward_dirs_[i] = outward_dir_for(B_[i], normals_[i / 2]);
            }
        }

        void set_platform_params(
            const double rb,
            const double l1,
            const double l2,
            const double rp,
            const double zrp,
            const double ei,
            const double bi)
        {
            rb_ = rb;
            l1_ = l1;
            l2_ = l2;
            rp_ = rp;
            zrp_ = zrp;
            ei_ = ei;
            bi_ = bi;
        }

        // ik
        std::array<double, 6> calculate(double x, double y, double z, double roll, double pitch, double yaw);
        // fk
        std::array<double, 6> calculate(const std::array<double, 6>& joint_angles);

    private:
        double kTol = 1e-12;

        double rb_ = 0.214;
        double l1_ = 0.12;
        double l2_ = 0.3075;
        double rp_ = 0.134;
        double zrp_ = 0.000;
        double ei_ = 0.1;
        double bi_ = 0.03;

        double base_ang_offset_{};
        double plat_ang_offset_{};

        std::array<Eigen::Vector3d, 6> B_{};
        std::array<Eigen::Vector3d, 6> pi_local_{};
        std::array<Eigen::Vector3d, 3> normals_{};
        std::array<double, 3> Dvals_{};
        std::array<Eigen::Vector3d, 6> outward_dirs_{};

        std::array<double, 6> phi_b_{};
        std::array<double, 6> lambda_p_{};

        Eigen::Vector3d outward_dir_for(const Eigen::Vector3d& Bp, const Eigen::Vector3d& n);


        static Eigen::Matrix3d rotation_from_rpy(double roll, double pitch, double yaw);

        std::array<Eigen::Vector3d, 6> crank_points_from_angles(const std::array<double, 6>& joint_angles) const;
        Eigen::Matrix<double, 6, 1> residual_vector(const Eigen::Matrix<double, 6, 1>& pose,
                                                    const std::array<Eigen::Vector3d, 6>& A) const;
        Eigen::Matrix<double, 6, 6> numerical_jacobian(const Eigen::Matrix<double, 6, 1>& pose,
                                                       const std::array<Eigen::Vector3d, 6>& A) const;

        static bool allFinite(const Eigen::Matrix<double, 6, 1>& v)
        {
            for (int i = 0; i < 6; ++i)
            {
                if (!std::isfinite(v(i))) return false;
            }
            return true;
        }

        static bool allFinite(const Eigen::Matrix<double, 6, 6>& M)
        {
            for (int r = 0; r < 6; ++r)
            {
                for (int c = 0; c < 6; ++c)
                {
                    if (!std::isfinite(M(r, c))) return false;
                }
            }
            return true;
        }
    };
}

#endif //STEWART_IK_HPP
