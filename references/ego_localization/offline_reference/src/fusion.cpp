#include "offline_fusion/fusion.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_fusion/dataset.hpp"
#include "offline_fusion/math.hpp"

namespace offline_fusion {

#ifdef OFFLINE_FUSION_HAS_CERES
std::vector<TrajectoryState> run_ceres_gins(const Dataset& dataset,
                                            const FusionOptions& options,
                                            FusionSummary* summary);
#endif

namespace {

constexpr int kStateDim = 15;

struct GraphNode {
    double t = 0.0;
    Vec3 nominal_position;
    Vec3 nominal_velocity;
    Quat nominal_q_body_to_ned;
    Vec3 gnss_position;
    Vec3 gnss_position_std{3.0, 3.0, 3.0};
};

struct ImuPreintegration {
    Vec3 delta_p;
    Vec3 delta_v;
    Vec3 delta_theta;
    double dt = 0.0;
    std::size_t samples = 0;
};

struct LinearFactor {
    std::vector<int> indices;
    std::vector<double> coeffs;
    double rhs = 0.0;
    double sigma = 1.0;
    std::string type;
};

double component(const Vec3& v, int dim) {
    if (dim == 0) {
        return v.x;
    }
    if (dim == 1) {
        return v.y;
    }
    return v.z;
}

Vec3 basis(int dim) {
    if (dim == 0) {
        return {1.0, 0.0, 0.0};
    }
    if (dim == 1) {
        return {0.0, 1.0, 0.0};
    }
    return {0.0, 0.0, 1.0};
}

bool in_range(double t, double start, double end) {
    return t >= start && t <= end;
}

int base_index(std::size_t node_index) {
    return static_cast<int>(node_index * kStateDim);
}

int p_index(std::size_t node_index, int dim) {
    return base_index(node_index) + dim;
}

int v_index(std::size_t node_index, int dim) {
    return base_index(node_index) + 3 + dim;
}

int theta_index(std::size_t node_index, int dim) {
    return base_index(node_index) + 6 + dim;
}

int ba_index(std::size_t node_index, int dim) {
    return base_index(node_index) + 9 + dim;
}

int bg_index(std::size_t node_index, int dim) {
    return base_index(node_index) + 12 + dim;
}

Vec3 vector_at(const std::vector<double>& x, int (*indexer)(std::size_t, int), std::size_t node) {
    return {
        x[static_cast<std::size_t>(indexer(node, 0))],
        x[static_cast<std::size_t>(indexer(node, 1))],
        x[static_cast<std::size_t>(indexer(node, 2))],
    };
}

void add_scalar_factor(std::vector<LinearFactor>& factors,
                       std::vector<int> indices,
                       std::vector<double> coeffs,
                       double rhs,
                       double sigma,
                       const std::string& type) {
    factors.push_back({std::move(indices), std::move(coeffs), rhs, sigma, type});
}

std::vector<GraphNode> build_graph_nodes(const Dataset& dataset,
                                         double start_time,
                                         double end_time) {
    std::vector<GraphNode> nodes;
    for (const auto& fix : dataset.gnss_position_fixes) {
        if (!in_range(fix.t, start_time, end_time)) {
            continue;
        }
        GraphNode node;
        node.t = fix.t;
        node.nominal_position = fix.position_ned;
        node.nominal_velocity = fix.velocity_ned;
        node.nominal_q_body_to_ned = nearest_groundtruth_orientation(dataset.groundtruth, fix.t);
        node.gnss_position = fix.position_ned;
        node.gnss_position_std = fix.position_std_ned;
        nodes.push_back(node);
    }
    if (nodes.size() < 2) {
        throw std::runtime_error("Need at least two GNSS epochs in the selected time range");
    }
    return nodes;
}

ImuPreintegration preintegrate_imu_interval(const Dataset& dataset,
                                            double start_time,
                                            double end_time,
                                            const Quat& q0_body_to_ned,
                                            double gravity_mps2) {
    ImuPreintegration pim;
    const Vec3 gravity_ned{0.0, 0.0, gravity_mps2};
    Quat q = q0_body_to_ned;

    for (std::size_t i = 1; i < dataset.imu.size(); ++i) {
        const auto& prev = dataset.imu[i - 1];
        const auto& cur = dataset.imu[i];
        if (cur.t <= start_time || prev.t >= end_time) {
            continue;
        }

        const double a = std::max(prev.t, start_time);
        const double b = std::min(cur.t, end_time);
        const double dt = b - a;
        if (dt <= 0.0 || dt > 0.1) {
            continue;
        }

        const Vec3 acc_ned = rotate(q, prev.accel) + gravity_ned;
        pim.delta_p = pim.delta_p + pim.delta_v * dt + acc_ned * (0.5 * dt * dt);
        pim.delta_v = pim.delta_v + acc_ned * dt;
        q = quat_normalized(quat_multiply(q, quat_from_delta_angle(prev.gyro * dt)));
        pim.dt += dt;
        ++pim.samples;
    }

    const Quat dq = quat_multiply(quat_conjugate(q0_body_to_ned), q);
    pim.delta_theta = quat_to_delta_angle(dq);
    return pim;
}

std::vector<double> initial_vector_from_nodes(const std::vector<GraphNode>& nodes) {
    std::vector<double> x(nodes.size() * kStateDim, 0.0);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        x[p_index(i, 0)] = nodes[i].nominal_position.x;
        x[p_index(i, 1)] = nodes[i].nominal_position.y;
        x[p_index(i, 2)] = nodes[i].nominal_position.z;
        x[v_index(i, 0)] = nodes[i].nominal_velocity.x;
        x[v_index(i, 1)] = nodes[i].nominal_velocity.y;
        x[v_index(i, 2)] = nodes[i].nominal_velocity.z;
    }
    return x;
}

double weighted_cost(const std::vector<LinearFactor>& factors, const std::vector<double>& x) {
    double cost = 0.0;
    for (const auto& factor : factors) {
        double residual = -factor.rhs;
        for (std::size_t k = 0; k < factor.indices.size(); ++k) {
            residual += factor.coeffs[k] * x[static_cast<std::size_t>(factor.indices[k])];
        }
        const double normalized = residual / factor.sigma;
        cost += 0.5 * normalized * normalized;
    }
    return cost;
}

std::vector<double> solve_dense_system(std::vector<std::vector<double>> a,
                                       std::vector<double> b) {
    const std::size_t n = b.size();
    for (std::size_t col = 0; col < n; ++col) {
        std::size_t pivot = col;
        double pivot_abs = std::abs(a[col][col]);
        for (std::size_t row = col + 1; row < n; ++row) {
            const double candidate = std::abs(a[row][col]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }

        if (pivot_abs < 1e-14) {
            throw std::runtime_error("Factor graph normal matrix is singular");
        }

        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }

        const double diag = a[col][col];
        for (std::size_t j = col; j < n; ++j) {
            a[col][j] /= diag;
        }
        b[col] /= diag;

        for (std::size_t row = 0; row < n; ++row) {
            if (row == col) {
                continue;
            }
            const double scale = a[row][col];
            if (std::abs(scale) < 1e-18) {
                continue;
            }
            for (std::size_t j = col; j < n; ++j) {
                a[row][j] -= scale * a[col][j];
            }
            b[row] -= scale * b[col];
        }
    }
    return b;
}

std::vector<double> solve_linear_factor_graph(const std::vector<LinearFactor>& factors,
                                              std::size_t variable_count,
                                              double damping) {
    std::vector<std::vector<double>> normal(variable_count,
                                            std::vector<double>(variable_count, 0.0));
    std::vector<double> rhs(variable_count, 0.0);

    for (const auto& factor : factors) {
        const double w = 1.0 / (factor.sigma * factor.sigma);
        for (std::size_t i = 0; i < factor.indices.size(); ++i) {
            const int row = factor.indices[i];
            rhs[static_cast<std::size_t>(row)] += w * factor.coeffs[i] * factor.rhs;
            for (std::size_t j = 0; j < factor.indices.size(); ++j) {
                const int col = factor.indices[j];
                normal[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] +=
                    w * factor.coeffs[i] * factor.coeffs[j];
            }
        }
    }

    for (std::size_t i = 0; i < variable_count; ++i) {
        normal[i][i] += damping;
    }

    return solve_dense_system(std::move(normal), std::move(rhs));
}

void add_vector_prior(std::vector<LinearFactor>& factors,
                      std::size_t node,
                      int (*indexer)(std::size_t, int),
                      const Vec3& value,
                      double sigma,
                      const std::string& type,
                      std::size_t* counter) {
    for (int dim = 0; dim < 3; ++dim) {
        add_scalar_factor(factors,
                          {indexer(node, dim)},
                          {1.0},
                          component(value, dim),
                          sigma,
                          type);
        if (counter != nullptr) {
            ++(*counter);
        }
    }
}

std::vector<LinearFactor> build_factors(const Dataset& dataset,
                                        const std::vector<GraphNode>& nodes,
                                        const FusionOptions& options,
                                        FusionSummary* summary) {
    std::vector<LinearFactor> factors;
    std::size_t prior_factors = 0;
    std::size_t gnss_factors = 0;
    std::size_t imu_factors = 0;
    std::size_t attitude_factors = 0;
    std::size_t bias_factors = 0;
    std::size_t imu_used = 0;

    add_vector_prior(factors, 0, p_index, nodes.front().nominal_position,
                     options.prior_position_std_m, "prior_position", &prior_factors);
    add_vector_prior(factors, 0, v_index, nodes.front().nominal_velocity,
                     options.prior_velocity_std_mps, "prior_velocity", &prior_factors);
    add_vector_prior(factors, 0, theta_index, {0.0, 0.0, 0.0},
                     options.attitude_prior_std_rad, "prior_attitude_error", &prior_factors);
    add_vector_prior(factors, 0, ba_index, {0.0, 0.0, 0.0},
                     options.accel_bias_prior_std_mps2, "prior_accel_bias", &prior_factors);
    add_vector_prior(factors, 0, bg_index, {0.0, 0.0, 0.0},
                     options.gyro_bias_prior_std_rps, "prior_gyro_bias", &prior_factors);

    for (std::size_t i = 0; i < nodes.size(); ++i) {
        for (int dim = 0; dim < 3; ++dim) {
            add_scalar_factor(factors,
                              {p_index(i, dim)},
                              {1.0},
                              component(nodes[i].gnss_position, dim),
                              std::max(0.15, std::min(
                                  options.gnss_position_std_m,
                                  component(nodes[i].gnss_position_std, dim))),
                              "gnss_raw_position");
            ++gnss_factors;
        }
    }

    for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
        const double dt = nodes[i + 1].t - nodes[i].t;
        const ImuPreintegration pim =
            preintegrate_imu_interval(dataset,
                                      nodes[i].t,
                                      nodes[i + 1].t,
                                      nodes[i].nominal_q_body_to_ned,
                                      options.gravity_mps2);
        imu_used += pim.samples;

        const Vec3 nominal_dp = nodes[i + 1].nominal_position -
                                nodes[i].nominal_position -
                                nodes[i].nominal_velocity * dt;
        const Vec3 nominal_dv = nodes[i + 1].nominal_velocity -
                                nodes[i].nominal_velocity;

        const Quat nominal_relative_q =
            quat_multiply(quat_conjugate(nodes[i].nominal_q_body_to_ned),
                          nodes[i + 1].nominal_q_body_to_ned);
        const Vec3 nominal_delta_theta = quat_to_delta_angle(nominal_relative_q);

        const Vec3 dp_bias_rhs = nominal_dp - pim.delta_p;
        const Vec3 dv_bias_rhs = nominal_dv - pim.delta_v;
        const Vec3 dtheta_rhs = nominal_delta_theta - pim.delta_theta;

        for (int dim = 0; dim < 3; ++dim) {
            const double r_ba_x = component(rotate(nodes[i].nominal_q_body_to_ned, basis(0)), dim);
            const double r_ba_y = component(rotate(nodes[i].nominal_q_body_to_ned, basis(1)), dim);
            const double r_ba_z = component(rotate(nodes[i].nominal_q_body_to_ned, basis(2)), dim);
            add_scalar_factor(factors,
                              {p_index(i + 1, dim), p_index(i, dim), v_index(i, dim),
                               ba_index(i, 0), ba_index(i, 1), ba_index(i, 2)},
                              {1.0, -1.0, -dt,
                               0.5 * dt * dt * r_ba_x,
                               0.5 * dt * dt * r_ba_y,
                               0.5 * dt * dt * r_ba_z},
                              component(pim.delta_p, dim),
                              options.imu_position_std_m,
                              "imu_position_preintegration");
            add_scalar_factor(factors,
                              {v_index(i + 1, dim), v_index(i, dim),
                               ba_index(i, 0), ba_index(i, 1), ba_index(i, 2)},
                              {1.0, -1.0,
                               dt * r_ba_x,
                               dt * r_ba_y,
                               dt * r_ba_z},
                              component(pim.delta_v, dim),
                              options.imu_velocity_std_mps,
                              "imu_velocity_preintegration");
            add_scalar_factor(factors,
                              {theta_index(i + 1, dim), theta_index(i, dim), bg_index(i, dim)},
                              {1.0, -1.0, dt},
                              component(pim.delta_theta, dim),
                              options.imu_attitude_std_rad,
                              "imu_attitude_preintegration");
            ++imu_factors;
            ++imu_factors;
            ++attitude_factors;

            (void)dp_bias_rhs;
            (void)dv_bias_rhs;
            (void)dtheta_rhs;
        }

        for (int dim = 0; dim < 3; ++dim) {
            add_scalar_factor(factors,
                              {ba_index(i + 1, dim), ba_index(i, dim)},
                              {1.0, -1.0},
                              0.0,
                              options.accel_bias_rw_std_mps2,
                              "accel_bias_random_walk");
            add_scalar_factor(factors,
                              {bg_index(i + 1, dim), bg_index(i, dim)},
                              {1.0, -1.0},
                              0.0,
                              options.gyro_bias_rw_std_rps,
                              "gyro_bias_random_walk");
            ++bias_factors;
            ++bias_factors;
        }
    }

    if (summary != nullptr) {
        summary->prior_factors = prior_factors;
        summary->gnss_factors = gnss_factors;
        summary->imu_factors = imu_factors;
        summary->attitude_factors = attitude_factors;
        summary->bias_factors = bias_factors;
        summary->imu_used = imu_used;
    }
    return factors;
}

Quat corrected_orientation(const Quat& nominal, const Vec3& theta_error) {
    return quat_normalized(quat_multiply(nominal, quat_from_delta_angle(theta_error)));
}

}  // namespace

std::vector<TrajectoryState> run_offline_gins(const Dataset& dataset,
                                              const FusionOptions& options,
                                              FusionSummary* summary) {
    if (dataset.imu.size() < 2 || dataset.gnss_position_fixes.empty()) {
        throw std::runtime_error("Dataset is too small for offline fusion");
    }

#ifdef OFFLINE_FUSION_HAS_CERES
    return run_ceres_gins(dataset, options, summary);
#else
    if (dataset.format == DatasetFormat::StructuredExperiment) {
        throw std::runtime_error(
            "Structured sensor initialization currently requires the Ceres build");
    }
    const double raw_start = std::max({dataset.imu.front().t,
                                       dataset.groundtruth.front().t,
                                       dataset.gnss_position_fixes.front().t});
    const double raw_end = std::min({dataset.imu.back().t,
                                     dataset.groundtruth.back().t,
                                     dataset.gnss_position_fixes.back().t});
    const double start_time = raw_start + options.start_offset_s;
    const double end_time = options.end_offset_s > 0.0 ? raw_start + options.end_offset_s : raw_end;
    if (start_time >= end_time) {
        throw std::runtime_error("Invalid fusion time range");
    }

    FusionSummary local_summary;
    local_summary.start_time = start_time;
    local_summary.end_time = end_time;

    const std::vector<GraphNode> nodes = build_graph_nodes(dataset, start_time, end_time);
    const std::vector<LinearFactor> factors = build_factors(dataset, nodes, options, &local_summary);
    const std::vector<double> initial = initial_vector_from_nodes(nodes);
    const std::vector<double> optimized =
        solve_linear_factor_graph(factors, initial.size(), options.normal_matrix_damping);

    local_summary.initial_cost = weighted_cost(factors, initial);
    local_summary.final_cost = weighted_cost(factors, optimized);
    local_summary.gnss_epochs_used = nodes.size();
    local_summary.state_nodes = nodes.size();
    local_summary.states_written = nodes.size();
    local_summary.gnss_fix_factors = 0;
    local_summary.gnss_spp_epochs_solved = dataset.gnss_spp_diagnostics.epochs_solved;
    local_summary.gnss_spp_epochs_rejected = dataset.gnss_spp_diagnostics.epochs_rejected;
    local_summary.gnss_spp_mean_satellites = dataset.gnss_spp_diagnostics.mean_satellites;
    local_summary.gnss_spp_mean_postfit_rms_m =
        dataset.gnss_spp_diagnostics.mean_postfit_rms_m;
    local_summary.gnss_spp_max_postfit_rms_m =
        dataset.gnss_spp_diagnostics.max_postfit_rms_m;
    local_summary.gnss_spp_min_weight =
        dataset.gnss_spp_diagnostics.min_normalized_weight;
    local_summary.gnss_spp_max_weight =
        dataset.gnss_spp_diagnostics.max_normalized_weight;
    local_summary.gnss_fixed_satellite_consistency_rms_m =
        dataset.gnss_spp_diagnostics.fixed_satellite_consistency_rms_m;
    local_summary.gnss_fixed_satellite_consistency_max_m =
        dataset.gnss_spp_diagnostics.fixed_satellite_consistency_max_m;
    local_summary.gnss_fixed_satellite_model_consistent =
        dataset.gnss_spp_diagnostics.fixed_satellite_model_consistent;
    local_summary.groundtruth_used_for_frame_anchor = true;

    std::vector<TrajectoryState> trajectory;
    trajectory.reserve(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const Vec3 theta_error = vector_at(optimized, theta_index, i);
        TrajectoryState state;
        state.t = nodes[i].t;
        state.position_ned = {
            optimized[p_index(i, 0)],
            optimized[p_index(i, 1)],
            optimized[p_index(i, 2)],
        };
        state.velocity_ned = {
            optimized[v_index(i, 0)],
            optimized[v_index(i, 1)],
            optimized[v_index(i, 2)],
        };
        state.q_body_to_ned = corrected_orientation(nodes[i].nominal_q_body_to_ned, theta_error);
        state.accel_bias = vector_at(optimized, ba_index, i);
        state.gyro_bias = vector_at(optimized, bg_index, i);
        state.update_source = "preintegrated_imu_factor_graph";
        trajectory.push_back(state);
    }

    if (summary != nullptr) {
        *summary = local_summary;
    }

    return trajectory;
#endif
}

void write_trajectory_csv(const std::filesystem::path& path,
                          const std::vector<TrajectoryState>& trajectory) {
    if (path.parent_path() != std::filesystem::path{}) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Failed to create " + path.string());
    }

    output << "timestamp_s,p_n_m,p_e_m,p_d_m,v_n_mps,v_e_mps,v_d_mps,"
           << "q_w,q_x,q_y,q_z,ba_x_mps2,ba_y_mps2,ba_z_mps2,"
           << "bg_x_rps,bg_y_rps,bg_z_rps,update_source\n";
    output << std::fixed << std::setprecision(9);
    for (const auto& state : trajectory) {
        output << state.t << ','
               << state.position_ned.x << ','
               << state.position_ned.y << ','
               << state.position_ned.z << ','
               << state.velocity_ned.x << ','
               << state.velocity_ned.y << ','
               << state.velocity_ned.z << ','
               << state.q_body_to_ned.w << ','
               << state.q_body_to_ned.x << ','
               << state.q_body_to_ned.y << ','
               << state.q_body_to_ned.z << ','
               << state.accel_bias.x << ','
               << state.accel_bias.y << ','
               << state.accel_bias.z << ','
               << state.gyro_bias.x << ','
               << state.gyro_bias.y << ','
               << state.gyro_bias.z << ','
               << state.update_source << '\n';
    }
}

void write_summary_txt(const std::filesystem::path& output_csv,
                       const FusionOptions& options,
                       const FusionSummary& summary,
                       const Dataset& dataset) {
    const std::filesystem::path summary_path = output_csv.parent_path() / "fusion_summary.txt";
    std::ofstream output(summary_path);
    if (!output) {
        throw std::runtime_error("Failed to create " + summary_path.string());
    }

    output << "Offline eVTOL fusion summary\n";
    output << "data_dir: " << options.data_dir.string() << '\n';
    output << "output_csv: " << output_csv.string() << '\n';
    output << "mode: offline IMU preintegration factor graph, state=(p_ned, v_ned, q_body_to_ned, ba, bg)\n";
    output << "note: GNSS SPP-like fixes use per-epoch pseudorange, azimuth, elevation, and SNR with fixed virtual satellites; no pseudorange differencing is used.\n";
    output << "note: gps_fix is loaded for interface and independent baseline evaluation only; it is never added to the estimator.\n";
    output << "note: carrier phase and Doppler are loaded but not used in this version.\n";
    output << "solver: " << summary.solver << '\n';
    output << "termination: " << summary.termination << '\n';
    output << "imu_samples_loaded: " << dataset.imu.size() << '\n';
    output << "gnss_epochs_loaded: " << dataset.gnss_epochs.size() << '\n';
    output << "gnss_position_fixes_loaded: " << dataset.gnss_position_fixes.size() << '\n';
    output << "groundtruth_samples_loaded: " << dataset.groundtruth.size() << '\n';
    output << "initialization_source: " << summary.initialization_source << '\n';
    output << "groundtruth_used_for_initialization: "
           << (summary.groundtruth_used_for_initialization ? "true" : "false") << '\n';
    output << "groundtruth_used_for_frame_anchor: "
           << (summary.groundtruth_used_for_frame_anchor ? "true" : "false") << '\n';
    output << "initialization_gravity_samples: "
           << summary.initialization_gravity_samples << '\n';
    output << "initialization_heading_time_s: "
           << summary.initialization_heading_time_s << '\n';
    output << "initialization_rpy_deg: "
           << summary.initialization_roll_deg << ","
           << summary.initialization_pitch_deg << ","
           << summary.initialization_yaw_deg << '\n';
    output << "start_time: " << std::fixed << std::setprecision(9) << summary.start_time << '\n';
    output << "end_time: " << std::fixed << std::setprecision(9) << summary.end_time << '\n';
    output << "state_nodes: " << summary.state_nodes << '\n';
    output << "states_written: " << summary.states_written << '\n';
    output << "imu_used: " << summary.imu_used << '\n';
    output << "gnss_epochs_used: " << summary.gnss_epochs_used << '\n';
    output << "prior_scalar_factors: " << summary.prior_factors << '\n';
    output << "gnss_scalar_factors: " << summary.gnss_factors << '\n';
    output << "gnss_fix_scalar_factors: " << summary.gnss_fix_factors << '\n';
    output << "gnss_spp_epochs_solved: " << summary.gnss_spp_epochs_solved << '\n';
    output << "gnss_spp_epochs_rejected: " << summary.gnss_spp_epochs_rejected << '\n';
    output << "gnss_spp_mean_satellites: " << summary.gnss_spp_mean_satellites << '\n';
    output << "gnss_spp_mean_postfit_rms_m: "
           << summary.gnss_spp_mean_postfit_rms_m << '\n';
    output << "gnss_spp_max_postfit_rms_m: "
           << summary.gnss_spp_max_postfit_rms_m << '\n';
    output << "gnss_spp_normalized_weight_range: "
           << summary.gnss_spp_min_weight << "," << summary.gnss_spp_max_weight << '\n';
    output << "gnss_fixed_satellite_consistency_rms_m: "
           << summary.gnss_fixed_satellite_consistency_rms_m << '\n';
    output << "gnss_fixed_satellite_consistency_max_m: "
           << summary.gnss_fixed_satellite_consistency_max_m << '\n';
    output << "gnss_fixed_satellite_model_consistent: "
           << (summary.gnss_fixed_satellite_model_consistent ? "true" : "false") << '\n';
    output << "gps_fix_role: loaded_interface_and_baseline_only\n";
    output << "carrier_phase_status: loaded_not_used\n";
    output << "doppler_status: loaded_not_used\n";
    output << "fiveg_enabled: " << (summary.fiveg_enabled ? "true" : "false") << '\n';
    output << "fiveg_epochs_used: " << summary.fiveg_epochs_used << '\n';
    output << "fiveg_range_factors: " << summary.fiveg_range_factors << '\n';
    output << "fiveg_rsrp_rejected: " << summary.fiveg_rsrp_rejected << '\n';
    output << "imu_scalar_factors: " << summary.imu_factors << '\n';
    output << "attitude_scalar_factors: " << summary.attitude_factors << '\n';
    output << "bias_scalar_factors: " << summary.bias_factors << '\n';
    output << "visual_enabled: " << (summary.visual_enabled ? "true" : "false") << '\n';
    output << "camera_frames_loaded: " << dataset.camera_frames.size() << '\n';
    output << "camera_frames_processed: " << summary.camera_frames_loaded << '\n';
    output << "visual_keyframes: " << summary.visual_keyframes << '\n';
    output << "visual_tracks: " << summary.visual_tracks << '\n';
    output << "visual_landmarks: " << summary.visual_landmarks << '\n';
    output << "visual_reprojection_factors: " << summary.visual_reprojection_factors << '\n';
    output << "visual_rejected_tracks: " << summary.visual_rejected_tracks << '\n';
    output << "visual_initial_reprojection_observations: "
           << summary.visual_initial_reprojection_observations << '\n';
    output << "visual_initial_reprojection_rmse_px: "
           << summary.visual_initial_reprojection_rmse_px << '\n';
    output << "lidar_enabled: " << (summary.lidar_enabled ? "true" : "false") << '\n';
    output << "lidar_frames_loaded: " << summary.lidar_frames_loaded << '\n';
    output << "lidar_keyframes: " << summary.lidar_keyframes << '\n';
    output << "lidar_constraints: " << summary.lidar_constraints << '\n';
    output << "lidar_rejected_pairs: " << summary.lidar_rejected_pairs << '\n';
    output << "lidar_correspondences: " << summary.lidar_correspondences << '\n';
    output << "lidar_mean_icp_rmse_m: " << summary.lidar_mean_icp_rmse_m << '\n';
    output << "lidar_mean_overlap: " << summary.lidar_mean_overlap << '\n';
    output << "lidar_mean_translation_std_m: "
           << summary.lidar_mean_translation_std_m << '\n';
    output << "lidar_mean_rotation_std_rad: "
           << summary.lidar_mean_rotation_std_rad << '\n';
    output << "initial_cost: " << summary.initial_cost << '\n';
    output << "final_cost: " << summary.final_cost << '\n';
    output << "gravity_mps2: " << options.gravity_mps2 << '\n';
    output << "initialization_gravity_window_s: "
           << options.initialization_gravity_window_s << '\n';
    output << "initialization_gravity_tolerance_mps2: "
           << options.initialization_gravity_tolerance_mps2 << '\n';
    output << "initialization_heading_speed_mps: "
           << options.initialization_heading_speed_mps << '\n';
    output << "prior_position_std_m: " << options.prior_position_std_m << '\n';
    output << "prior_velocity_std_mps: " << options.prior_velocity_std_mps << '\n';
    output << "attitude_prior_std_rad: " << options.attitude_prior_std_rad << '\n';
    output << "accel_bias_prior_std_mps2: " << options.accel_bias_prior_std_mps2 << '\n';
    output << "gyro_bias_prior_std_rps: " << options.gyro_bias_prior_std_rps << '\n';
    output << "gnss_position_std_m: " << options.gnss_position_std_m << '\n';
    output << "gnss_position_noise_scale: " << options.gnss_position_noise_scale << '\n';
    output << "use_gnss_fix_requested: false\n";
    output << "use_fiveg_requested: " << (options.use_fiveg ? "true" : "false") << '\n';
    output << "fiveg_range_std_floor_m: " << options.fiveg_range_std_floor_m << '\n';
    output << "fiveg_loss_scale: " << options.fiveg_loss_scale << '\n';
    output << "fiveg_min_prs_rsrp_dbm: " << options.fiveg_min_prs_rsrp_dbm << '\n';
    output << "imu_propagated_covariance: " << (options.imu_propagated_covariance ? "true" : "false") << '\n';
    output << "imu_accel_noise_density: " << options.imu_accel_noise_density << '\n';
    output << "imu_gyro_noise_density: " << options.imu_gyro_noise_density << '\n';
    output << "imu_accel_bias_density: " << options.imu_accel_bias_density << '\n';
    output << "imu_gyro_bias_density: " << options.imu_gyro_bias_density << '\n';
    output << "extra_state_interval_s: " << options.extra_state_interval_s << '\n';
    output << "imu_position_std_m: " << options.imu_position_std_m << '\n';
    output << "imu_velocity_std_mps: " << options.imu_velocity_std_mps << '\n';
    output << "imu_attitude_std_rad: " << options.imu_attitude_std_rad << '\n';
    output << "accel_bias_rw_std_mps2: " << options.accel_bias_rw_std_mps2 << '\n';
    output << "gyro_bias_rw_std_rps: " << options.gyro_bias_rw_std_rps << '\n';
    output << "normal_matrix_damping: " << options.normal_matrix_damping << '\n';
    output << "use_visual_requested: " << (options.use_visual ? "true" : "false") << '\n';
    output << "visual_keyframe_interval_s: " << options.visual_keyframe_interval_s << '\n';
    output << "visual_max_features: " << options.visual_max_features << '\n';
    output << "visual_max_landmarks: " << options.visual_max_landmarks << '\n';
    output << "visual_max_observations_per_landmark: "
           << options.visual_max_observations_per_landmark << '\n';
    output << "visual_min_parallax_px: " << options.visual_min_parallax_px << '\n';
    output << "visual_reprojection_std_px: " << options.visual_reprojection_std_px << '\n';
    output << "visual_max_initial_reprojection_px: "
           << options.visual_max_initial_reprojection_px << '\n';
    output << "visual_image_scale: " << options.visual_image_scale << '\n';
    output << "visual_prune_unconstrained_nodes: "
           << (options.visual_prune_unconstrained_nodes ? "true" : "false") << '\n';
    output << "camera_pitch_deg: " << options.camera_pitch_deg << '\n';
    output << "absolute_start_s: " << options.absolute_start_s << '\n';
    output << "absolute_end_s: " << options.absolute_end_s << '\n';
    output << "solver_max_iterations: " << options.solver_max_iterations << '\n';
    output << "visual_min_triangulation_deg: " << options.visual_min_triangulation_deg << '\n';
    output << "lidar_min_information_ratio: " << options.lidar_min_information_ratio << '\n';
    output << "camera_fx: " << options.camera_fx << '\n';
    output << "camera_fy: " << options.camera_fy << '\n';
    output << "camera_cx: " << options.camera_cx << '\n';
    output << "camera_cy: " << options.camera_cy << '\n';
    output << "camera_width: " << options.camera_width << '\n';
    output << "camera_height: " << options.camera_height << '\n';
    output << "camera_t_body_camera_m: "
           << options.camera_t_body_camera.x << ","
           << options.camera_t_body_camera.y << ","
           << options.camera_t_body_camera.z << '\n';
    output << "use_lidar_requested: " << (options.use_lidar ? "true" : "false") << '\n';
    output << "lidar_keyframe_interval_s: " << options.lidar_keyframe_interval_s << '\n';
    output << "lidar_voxel_size_m: " << options.lidar_voxel_size_m << '\n';
    output << "lidar_range_m: " << options.lidar_min_range_m << ","
           << options.lidar_max_range_m << '\n';
    output << "lidar_max_points: " << options.lidar_max_points << '\n';
    output << "lidar_icp_max_iterations: " << options.lidar_icp_max_iterations << '\n';
    output << "lidar_min_correspondences: " << options.lidar_min_correspondences << '\n';
    output << "lidar_max_correspondence_distance_m: "
           << options.lidar_max_correspondence_distance_m << '\n';
    output << "lidar_min_overlap: " << options.lidar_min_overlap << '\n';
    output << "lidar_max_icp_rmse_m: " << options.lidar_max_icp_rmse_m << '\n';
    output << "lidar_max_correction_translation_m: "
           << options.lidar_max_correction_translation_m << '\n';
    output << "lidar_max_correction_rotation_rad: "
           << options.lidar_max_correction_rotation_rad << '\n';
    output << "lidar_max_correction_ratio: "
           << options.lidar_max_correction_ratio << '\n';
    output << "lidar_adaptive_covariance: "
           << (options.lidar_adaptive_covariance ? "true" : "false") << '\n';
    output << "lidar_adaptive_max_scale: "
           << options.lidar_adaptive_max_scale << '\n';
    output << "lidar_prune_unconstrained_nodes: "
           << (options.lidar_prune_unconstrained_nodes ? "true" : "false") << '\n';
    output << "lidar_translation_std_m: " << options.lidar_translation_std_m << '\n';
    output << "lidar_rotation_std_rad: " << options.lidar_rotation_std_rad << '\n';
    output << "lidar_t_body_lidar_m: " << options.lidar_t_body_lidar.x << ","
           << options.lidar_t_body_lidar.y << ","
           << options.lidar_t_body_lidar.z << '\n';
    output << "lidar_input_frame: "
           << (options.lidar_points_vehicle_inertial_frame ? "vehicle-inertial" : "sensor") << '\n';
}

}  // namespace offline_fusion
