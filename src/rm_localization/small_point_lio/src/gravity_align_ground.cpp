/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

// ============================================================================
// gravity_align_ground.cpp —— 本仓新增（2026-10-21，契约 v3 §4.3）
//
// 只被 `gravity_align_z_origin:=ground` 这一档调用（默认 `sensor` ⇒ 本文件不参与）。
// 见头文件顶部的"为什么独立 TU"。
// ============================================================================

#include "gravity_align_ground.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace small_point_lio {

    double z_origin_state_z(double h, const Eigen::Matrix3d &R_ws, const Eigen::Vector3d &t_IL) {
        // 传感器(livox)原点在状态(IMU body)系里的位置 = t_IL（= extrinsic_T，默认 (0,0,0.05)）。
        // 它在世界系里的**高度** = (R_ws·t_IL).z（车体水平时对 robot11 = 0.05·cos30° = 0.043301）。
        // 要 livox 恰好在 +h ⇒ 状态原点高度 = h − 该杆臂高度。
        return h - (R_ws * t_IL).z();
    }

    GroundFitResult fit_ground_plane_sensor_frame(
            const std::deque<common::Point> &points,
            double max_dt_s,
            const Eigen::Matrix3d &R_ws,
            const Eigen::Matrix3d &R_IL) {
        GroundFitResult out;

        // ---------- 0) 选点：最后一帧 + 合理的距离环 ----------
        double ts_max = -1e300;
        for (const auto &p: points) {
            ts_max = std::max(ts_max, p.timestamp);
        }
        std::vector<Eigen::Vector3d> P;
        P.reserve(points.size());
        for (const auto &p: points) {
            if (max_dt_s > 0.0 && p.timestamp < ts_max - max_dt_s) {
                continue;
            }
            const Eigen::Vector3d v = p.position.cast<double>();
            const double r = v.norm();
            // 排除自击（robot11 自击在 r<0.24 m）与太远的稀疏点（r>8 m 地面点密度低、曲率影响大）。
            if (r < 0.5 || r > 8.0) {
                continue;
            }
            P.push_back(v);
        }
        if (P.size() < 500) {
            return out;
        }

        // ---------- 1) 法向先验：world up 在 lidar 系 ----------
        const Eigen::Vector3d n_pred = (R_IL.transpose() * R_ws.transpose() * Eigen::Vector3d::UnitZ()).normalized();
        const double cos_tol = std::cos(20.0 * M_PI / 180.0);

        // ---------- 2) RANSAC（固定种子 = 确定性；子采样上限 2000 点）----------
        constexpr int kIters = 200;
        constexpr double kTol = 0.02;   // 2 cm：与 linefit 的 max_fit_error 0.05 同一物理量级，更紧
        constexpr double kDMin = 0.05;  // 传感器离地高度的物理先验（本仓三种模型 0.235–0.80）
        constexpr double kDMax = 0.60;
        const size_t stride = std::max<size_t>(1, P.size() / 2000);
        std::vector<Eigen::Vector3d> S;
        S.reserve(P.size() / stride + 1);
        for (size_t i = 0; i < P.size(); i += stride) {
            S.push_back(P[i]);
        }
        std::mt19937 rng(20261021u);
        std::uniform_int_distribution<size_t> pick(0, S.size() - 1);

        bool have = false;
        Eigen::Vector3d best_n = Eigen::Vector3d::UnitZ();
        double best_d = 0.0;
        int best_inl = 0;
        for (int it = 0; it < kIters; ++it) {
            const size_t i0 = pick(rng), i1 = pick(rng), i2 = pick(rng);
            if (i0 == i1 || i1 == i2 || i0 == i2) {
                continue;
            }
            Eigen::Vector3d n = (S[i1] - S[i0]).cross(S[i2] - S[i0]);
            const double nn = n.norm();
            if (nn < 1e-9) {
                continue;
            }
            n /= nn;
            double d = -n.dot(S[i0]);
            if (n.dot(n_pred) < 0.0) {  // 法向统一朝"上"
                n = -n;
                d = -d;
            }
            if (n.dot(n_pred) < cos_tol || d < kDMin || d > kDMax) {
                continue;
            }
            int inl = 0;
            for (const auto &q: S) {
                if (std::fabs(n.dot(q) + d) < kTol) {
                    ++inl;
                }
            }
            if (inl > best_inl) {
                best_inl = inl;
                best_n = n;
                best_d = d;
                have = true;
            }
        }
        if (!have || best_inl < 100) {
            return out;
        }

        // ---------- 3) 全点内点集上做最小二乘精修（SVD）----------
        std::vector<Eigen::Vector3d> Q;
        Q.reserve(P.size());
        for (const auto &q: P) {
            if (std::fabs(best_n.dot(q) + best_d) < kTol) {
                Q.push_back(q);
            }
        }
        if (Q.size() < 300) {
            return out;
        }
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const auto &q: Q) {
            c += q;
        }
        c /= static_cast<double>(Q.size());
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto &q: Q) {
            const Eigen::Vector3d v = q - c;
            cov += v * v.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
        Eigen::Vector3d n = eig.eigenvectors().col(0);  // 最小特征值方向 = 法向
        double d = -n.dot(c);
        if (n.dot(n_pred) < 0.0) {
            n = -n;
            d = -d;
        }
        if (d < kDMin || d > kDMax) {
            return out;
        }

        // ---------- 4) 统计（内点残差 + 倾角）----------
        std::vector<double> res;
        res.reserve(P.size());
        int inl = 0;
        for (const auto &q: P) {
            const double e = std::fabs(n.dot(q) + d);
            if (e < kTol) {
                ++inl;
                res.push_back(e);
            }
        }
        if (inl < 300) {
            return out;
        }
        std::sort(res.begin(), res.end());
        const auto pct = [&res](double p) {
            if (res.empty()) {
                return 0.0;
            }
            const size_t i = std::min(res.size() - 1, static_cast<size_t>(p * static_cast<double>(res.size() - 1)));
            return res[i];
        };
        const double tilt = std::acos(std::clamp(std::fabs(n.dot(n_pred)), 0.0, 1.0)) * 180.0 / M_PI;

        out.ok = tilt < 10.0;  // 法向与"重力方向"差 >10° ⇒ 拟到的是墙/坡，不是地面
        out.height_m = d;
        out.tilt_deg = tilt;
        out.inliers = inl;
        out.resid_p50_m = pct(0.50);
        out.resid_p95_m = pct(0.95);
        return out;
    }

}// namespace small_point_lio
