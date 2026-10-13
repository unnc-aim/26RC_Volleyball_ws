//
// Created by happydog on 6/3/26.
//
#include "volleyball_controller/stewart_ik.hpp"
#include <algorithm>
#include <limits>

using namespace aim::algorithm::ik::stewart;

std::array<double, 6> StewartIk::calculate(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Cartesian Space Limiting
    roll = std::clamp(roll, -0.3, 0.3);
    pitch = std::clamp(pitch, -0.3, 0.3);

    Eigen::Vector3d Translate(x, y, z);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    std::array<Eigen::Vector3d, 6> P;
    std::array<Eigen::Vector3d, 6> A;
    std::array<double, 6> joint_angles;

    // calculate P
    for (int i = 0; i < 6; ++i)
    {
        P[i] = Translate + R * pi_local_[i];
    }

    constexpr double kMinJointAngle = -40.0 * M_PI / 180.0;
    constexpr double kMaxJointAngle = 70.0 * M_PI / 180.0;

    auto outward_dir_for = [](const Eigen::Vector3d& Bp,
                              const Eigen::Vector3d& n) -> Eigen::Vector3d
    {
        // Crank zero definition: horizontal and pointing outward from the base center.
        // Because each servo crank moves in its own vertical plane, project the radial
        // outward direction onto that plane.
        Eigen::Vector3d radial(Bp.x(), Bp.y(), 0.0);
        if (radial.norm() < 1e-12)
        {
            radial = Eigen::Vector3d(1.0, 0.0, 0.0);
        }
        else
        {
            radial.normalize();
        }

        Eigen::Vector3d outward = radial - radial.dot(n) * n;
        outward.z() = 0.0;
        if (outward.norm() < 1e-12)
        {
            outward = Eigen::Vector3d(-n.y(), n.x(), 0.0);
            if (outward.dot(radial) < 0.0)
            {
                outward = -outward;
            }
        }
        return outward.normalized();
    };

    joint_angles.fill(std::numeric_limits<double>::quiet_NaN());

    for (int i = 0; i < 6; ++i)
    {
        int idx = i / 2;
        Eigen::Vector3d n = normals_[idx];
        double D = Dvals_[idx];
        Eigen::Vector3d Bp = B_[i];

        const double signed_plane_distance = n.dot(P[i]) + D;
        Eigen::Vector3d Pp = P[i] - signed_plane_distance * n;

        double d_plane = std::abs(signed_plane_distance);
        double l2_proj_sq = l2_ * l2_ - d_plane * d_plane;
        if (l2_proj_sq < -kTol)
        {
            continue;
        }
        double d_proj = std::sqrt(std::max(0.0, l2_proj_sq));

        Eigen::Vector3d vBP = Pp - Bp;
        double dBP = vBP.norm();
        if (dBP < 1e-12)
        {
            continue;
        }
        Eigen::Vector3d dirBP = vBP / dBP;

        Eigen::Vector3d perp_raw = n.cross(dirBP);
        double perp_norm = perp_raw.norm();
        if (perp_norm < 1e-12)
        {
            continue;
        }
        Eigen::Vector3d perp = perp_raw / perp_norm;

        double cos_t_raw = (l1_ * l1_ + dBP * dBP - d_proj * d_proj) / (2.0 * l1_ * dBP);
        if (cos_t_raw < -1.0 - kTol || cos_t_raw > 1.0 + kTol)
        {
            continue;
        }
        double cos_t = std::clamp(cos_t_raw, -1.0, 1.0);
        double theta = std::acos(cos_t);

        Eigen::Vector3d A1 = Bp + l1_ * (std::cos(theta) * dirBP + std::sin(theta) * perp);
        Eigen::Vector3d A2 = Bp + l1_ * (std::cos(theta) * dirBP - std::sin(theta) * perp);

        Eigen::Vector3d outward_dir = outward_dir_for(Bp, n);

        auto evaluate_candidate = [&](const Eigen::Vector3d& Ac,
                                      double& angle,
                                      double& score) -> bool
        {
            Eigen::Vector3d BA = Ac - Bp;
            double outward_component = BA.dot(outward_dir);
            double z_component = BA.z();

            // Servo angle convention:
            //   0 deg: crank is horizontal and points outward, parallel to base;
            //   positive: crank lifts upward along +Z;
            //   negative: crank moves downward along -Z.
            angle = std::atan2(z_component, outward_component);

            const bool points_outward = outward_component > 0.0;
            const bool in_range = angle >= kMinJointAngle - kTol && angle <= kMaxJointAngle + kTol;

            // This stateless API has no previous angle. When both branches are valid,
            // choose the branch closer to the mechanical zero instead of a crossing branch.
            score = std::abs(angle);
            return points_outward && in_range;
        };

        double angle1 = 0.0, angle2 = 0.0, score1 = 0.0, score2 = 0.0;
        const bool ok1 = evaluate_candidate(A1, angle1, score1);
        const bool ok2 = evaluate_candidate(A2, angle2, score2);

        if (ok1 && ok2)
        {
            if (score1 <= score2)
            {
                A[i] = A1;
                joint_angles[i] = angle1;
            }
            else
            {
                A[i] = A2;
                joint_angles[i] = angle2;
            }
        }
        else if (ok1)
        {
            A[i] = A1;
            joint_angles[i] = angle1;
        }
        else if (ok2)
        {
            A[i] = A2;
            joint_angles[i] = angle2;
        }
    }

    return joint_angles;
}

Eigen::Vector3d StewartIk::outward_dir_for(const Eigen::Vector3d& Bp, const Eigen::Vector3d& n)
{
    Eigen::Vector3d radial(Bp.x(), Bp.y(), 0.0);
    if (radial.norm() < kTol)
    {
        radial = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    else
    {
        radial.normalize();
    }

    Eigen::Vector3d outward = radial - radial.dot(n) * n;
    outward.z() = 0.0;
    if (outward.norm() < kTol)
    {
        outward = Eigen::Vector3d(-n.y(), n.x(), 0.0);
        if (outward.dot(radial) < 0.0)
        {
            outward = -outward;
        }
    }
    return outward.normalized();
}

Eigen::Matrix3d StewartIk::rotation_from_rpy(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return R;
}

std::array<Eigen::Vector3d, 6> StewartIk::crank_points_from_angles(const std::array<double, 6>& joint_angles) const
{
    std::array<Eigen::Vector3d, 6> A{};
    for (int i = 0; i < 6; ++i)
    {
        // 0 rad: horizontal outward; +rad: upward along +Z; -rad: downward along -Z.
        A[i] = B_[i] + l1_ * (std::cos(joint_angles[i]) * outward_dirs_[i]
            + std::sin(joint_angles[i]) * Eigen::Vector3d::UnitZ());
    }
    return A;
}

Eigen::Matrix<double, 6, 1> StewartIk::residual_vector(const Eigen::Matrix<double, 6, 1>& pose,
                                                       const std::array<Eigen::Vector3d, 6>& A) const
{
    Eigen::Matrix<double, 6, 1> r;
    const Eigen::Vector3d T(pose(0), pose(1), pose(2));
    const Eigen::Matrix3d R = rotation_from_rpy(pose(3), pose(4), pose(5));

    for (int i = 0; i < 6; ++i)
    {
        const Eigen::Vector3d P = T + R * pi_local_[i];
        r(i) = (P - A[i]).norm() - l2_;
    }
    return r;
}

Eigen::Matrix<double, 6, 6> StewartIk::numerical_jacobian(const Eigen::Matrix<double, 6, 1>& pose,
                                                          const std::array<Eigen::Vector3d, 6>& A) const
{
    Eigen::Matrix<double, 6, 6> J;
    J.setZero();

    for (int c = 0; c < 6; ++c)
    {
        const double h = (c < 3) ? 1e-6 : 1e-6;
        Eigen::Matrix<double, 6, 1> qp = pose;
        Eigen::Matrix<double, 6, 1> qm = pose;
        qp(c) += h;
        qm(c) -= h;

        const Eigen::Matrix<double, 6, 1> rp = residual_vector(qp, A);
        const Eigen::Matrix<double, 6, 1> rm = residual_vector(qm, A);
        J.col(c) = (rp - rm) / (2.0 * h);
    }

    return J;
}

std::array<double, 6> StewartIk::calculate(const std::array<double, 6>& joint_angles)
{
    constexpr int max_iterations = 80;
    constexpr double tolerance = 1e-8;

    Eigen::Matrix<double, 6, 1> q;
    const std::array<Eigen::Vector3d, 6> A = crank_points_from_angles(joint_angles);
    Eigen::Matrix<double, 6, 1> r = residual_vector(q, A);

    double norm = r.norm();
    double max_abs = r.cwiseAbs().maxCoeff();

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        if (max_abs <= tolerance)
        {
            break;
        }

        const Eigen::Matrix<double, 6, 6> J = numerical_jacobian(q, A);

        // Damped least-squares Newton step. It is more stable around singular poses
        // than an undamped inverse while still preserving Newton behavior near the root.
        const double lambda = 1e-6;
        const Eigen::Matrix<double, 6, 6> H = J.transpose() * J
            + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 1> g = J.transpose() * r;
        Eigen::Matrix<double, 6, 1> step = H.ldlt().solve(-g);

        // Trust-region style clipping. FK is local and multi-solution; very large steps
        // usually jump to another branch or make Euler angles ill-conditioned.
        double scale = 1.0;
        const double trans_norm = step.head<3>().norm();
        const double rot_norm = step.tail<3>().norm();
        if (trans_norm > 0.05) scale = std::min(scale, 0.05 / trans_norm);
        if (rot_norm > 0.20) scale = std::min(scale, 0.20 / rot_norm);
        step *= scale;

        bool accepted = false;
        Eigen::Matrix<double, 6, 1> best_q = q;
        Eigen::Matrix<double, 6, 1> best_r = r;
        double best_norm = norm;
        double alpha = 1.0;

        for (int line_search = 0; line_search < 16; ++line_search)
        {
            const Eigen::Matrix<double, 6, 1> q_try = q + alpha * step;
            const Eigen::Matrix<double, 6, 1> r_try = residual_vector(q_try, A);
            if (allFinite(r_try))
            {
                const double try_norm = r_try.norm();
                if (try_norm < best_norm)
                {
                    best_norm = try_norm;
                    best_q = q_try;
                    best_r = r_try;
                    accepted = true;
                    break;
                }
            }
            alpha *= 0.5;
        }

        if (!accepted)
        {
            break;
        }

        q = best_q;
        r = best_r;
        norm = best_norm;
        max_abs = r.cwiseAbs().maxCoeff();

        if ((alpha * step).norm() < 1e-12 && max_abs > tolerance)
        {
            break;
        }
    }

    std::array<double, 6> res_pose;
    for (int i = 0; i < 6; ++i)
    {
        res_pose[i] = q(i);
    }

    return res_pose;
}
