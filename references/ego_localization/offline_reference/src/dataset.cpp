#include "offline_fusion/dataset.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <random>
#include <cstdlib>

#include "offline_fusion/math.hpp"

namespace offline_fusion {
namespace {

bool is_data_line(const std::string& line) {
    for (char c : line) {
        if (c == '#') {
            return false;
        }
        if (!std::isspace(static_cast<unsigned char>(c))) {
            return true;
        }
    }
    return false;
}

template <typename T>
void ensure_nonempty(const std::vector<T>& values, const std::filesystem::path& path) {
    if (values.empty()) {
        throw std::runtime_error("No data loaded from " + path.string());
    }
}

std::vector<std::string> parse_csv_row(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char value = line[i];
        if (value == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (value == ',' && !quoted) {
            fields.push_back(field);
            field.clear();
        } else if (value != '\r') {
            field.push_back(value);
        }
    }
    fields.push_back(field);
    return fields;
}

class CsvTableReader {
public:
    explicit CsvTableReader(const std::filesystem::path& path) : input_(path), path_(path) {
        if (!input_) {
            throw std::runtime_error("Failed to open " + path.string());
        }
        std::string header_line;
        if (!std::getline(input_, header_line)) {
            throw std::runtime_error("Missing CSV header in " + path.string());
        }
        const auto header = parse_csv_row(header_line);
        for (std::size_t i = 0; i < header.size(); ++i) {
            columns_[header[i]] = i;
        }
    }

    bool next() {
        std::string line;
        while (std::getline(input_, line)) {
            if (line.empty()) {
                continue;
            }
            fields_ = parse_csv_row(line);
            return true;
        }
        return false;
    }

    const std::string& get(const std::string& name) const {
        const auto it = columns_.find(name);
        if (it == columns_.end()) {
            throw std::runtime_error("Missing CSV column " + name + " in " + path_.string());
        }
        if (it->second >= fields_.size()) {
            throw std::runtime_error("Short CSV row in " + path_.string());
        }
        return fields_[it->second];
    }

    bool has(const std::string& name) const {
        return columns_.find(name) != columns_.end();
    }

private:
    std::ifstream input_;
    std::filesystem::path path_;
    std::unordered_map<std::string, std::size_t> columns_;
    std::vector<std::string> fields_;
};

double relative_seconds(std::int64_t timestamp_ns, std::int64_t origin_ns) {
    return static_cast<double>(timestamp_ns - origin_ns) * 1e-9;
}

double structured_time_seconds(const CsvTableReader& reader,
                               const std::string& timestamp_column,
                               std::int64_t origin_ns) {
    if (reader.has("elapsed_s")) {
        return std::stod(reader.get("elapsed_s"));
    }
    return relative_seconds(std::stoll(reader.get(timestamp_column)), origin_ns);
}

bool read_elapsed_time_origin(const std::filesystem::path& path,
                              const std::string& timestamp_column,
                              std::int64_t* origin_ns) {
    CsvTableReader reader(path);
    if (!reader.next() || !reader.has("elapsed_s")) {
        return false;
    }
    const std::int64_t timestamp_ns = std::stoll(reader.get(timestamp_column));
    const double elapsed_s = std::stod(reader.get("elapsed_s"));
    *origin_ns = timestamp_ns -
        static_cast<std::int64_t>(std::llround(elapsed_s * 1e9));
    return true;
}

Vec3 geodetic_to_ecef(double latitude_deg, double longitude_deg, double altitude_m) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kSemiMajorAxisM = 6378137.0;
    constexpr double kEccentricitySquared = 6.69437999014e-3;
    const double latitude = latitude_deg * kPi / 180.0;
    const double longitude = longitude_deg * kPi / 180.0;
    const double sin_latitude = std::sin(latitude);
    const double cos_latitude = std::cos(latitude);
    const double sin_longitude = std::sin(longitude);
    const double cos_longitude = std::cos(longitude);
    const double prime_vertical_radius = kSemiMajorAxisM /
        std::sqrt(1.0 - kEccentricitySquared * sin_latitude * sin_latitude);
    return {
        (prime_vertical_radius + altitude_m) * cos_latitude * cos_longitude,
        (prime_vertical_radius + altitude_m) * cos_latitude * sin_longitude,
        (prime_vertical_radius * (1.0 - kEccentricitySquared) + altitude_m) *
            sin_latitude,
    };
}

Vec3 geodetic_to_ned(double latitude_deg,
                     double longitude_deg,
                     double altitude_m,
                     const GeodeticOrigin& origin) {
    if (!origin.valid) {
        throw std::runtime_error("Cannot convert GNSS fix without a geodetic origin");
    }
    constexpr double kPi = 3.14159265358979323846;
    const Vec3 ecef = geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m);
    const Vec3 origin_ecef =
        geodetic_to_ecef(origin.latitude_deg, origin.longitude_deg, origin.altitude_m);
    const Vec3 delta = ecef - origin_ecef;
    const double latitude = origin.latitude_deg * kPi / 180.0;
    const double longitude = origin.longitude_deg * kPi / 180.0;
    const double sin_latitude = std::sin(latitude);
    const double cos_latitude = std::cos(latitude);
    const double sin_longitude = std::sin(longitude);
    const double cos_longitude = std::cos(longitude);
    return {
        -sin_latitude * cos_longitude * delta.x -
            sin_latitude * sin_longitude * delta.y + cos_latitude * delta.z,
        -sin_longitude * delta.x + cos_longitude * delta.y,
        -cos_latitude * cos_longitude * delta.x -
            cos_latitude * sin_longitude * delta.y - sin_latitude * delta.z,
    };
}

bool parse_bool(const std::string& value) {
    return value == "1" || value == "true" || value == "True";
}

std::int64_t read_first_timestamp(const std::filesystem::path& path,
                                  const std::string& timestamp_column) {
    CsvTableReader reader(path);
    if (!reader.next()) {
        throw std::runtime_error("No CSV data rows in " + path.string());
    }
    return std::stoll(reader.get(timestamp_column));
}

Vec3 line_of_sight_ned(double elevation_deg, double azimuth_deg) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double elevation = elevation_deg * kDegToRad;
    const double azimuth = azimuth_deg * kDegToRad;
    const double horizontal = std::cos(elevation);
    return {
        horizontal * std::cos(azimuth),
        horizontal * std::sin(azimuth),
        -std::sin(elevation),
    };
}

std::array<double, 4> solve_4x4(std::array<std::array<double, 4>, 4> a,
                                std::array<double, 4> b) {
    for (std::size_t col = 0; col < 4; ++col) {
        std::size_t pivot = col;
        double pivot_abs = std::abs(a[col][col]);
        for (std::size_t row = col + 1; row < 4; ++row) {
            const double candidate = std::abs(a[row][col]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }
        if (pivot_abs < 1e-12) {
            throw std::runtime_error("GNSS raw normal matrix is singular");
        }
        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }

        const double diagonal = a[col][col];
        for (std::size_t j = col; j < 4; ++j) {
            a[col][j] /= diagonal;
        }
        b[col] /= diagonal;

        for (std::size_t row = 0; row < 4; ++row) {
            if (row == col) {
                continue;
            }
            const double scale = a[row][col];
            if (std::abs(scale) < 1e-18) {
                continue;
            }
            for (std::size_t j = col; j < 4; ++j) {
                a[row][j] -= scale * a[col][j];
            }
            b[row] -= scale * b[col];
        }
    }
    return b;
}

}  // namespace

std::vector<ImuSample> load_imu(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open " + path.string());
    }

    std::vector<ImuSample> samples;
    std::string line;
    while (std::getline(input, line)) {
        if (!is_data_line(line)) {
            continue;
        }
        std::istringstream iss(line);
        ImuSample sample;
        if (iss >> sample.t >> sample.accel.x >> sample.accel.y >> sample.accel.z >>
            sample.gyro.x >> sample.gyro.y >> sample.gyro.z) {
            samples.push_back(sample);
        }
    }
    ensure_nonempty(samples, path);
    return samples;
}

std::vector<GroundTruthSample> load_groundtruth(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open " + path.string());
    }

    std::vector<GroundTruthSample> samples;
    std::string line;
    while (std::getline(input, line)) {
        if (!is_data_line(line)) {
            continue;
        }
        std::istringstream iss(line);
        GroundTruthSample sample;
        if (iss >> sample.t >> sample.position_ned.x >> sample.position_ned.y >>
            sample.position_ned.z >> sample.q_body_to_ned.w >> sample.q_body_to_ned.x >>
            sample.q_body_to_ned.y >> sample.q_body_to_ned.z) {
            sample.q_body_to_ned = quat_normalized(sample.q_body_to_ned);
            sample.has_orientation = true;
            samples.push_back(sample);
        }
    }
    ensure_nonempty(samples, path);
    return samples;
}

std::vector<GnssEpoch> load_gnss_raw(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Failed to open " + path.string());
    }

    std::vector<GnssEpoch> epochs;
    std::string line;
    while (std::getline(input, line)) {
        if (!is_data_line(line)) {
            continue;
        }

        std::istringstream iss(line);
        double t = 0.0;
        GnssObservation obs;
        if (!(iss >> t >> obs.prn >> obs.pseudorange_m >> obs.carrier_phase_cyc >>
              obs.doppler_hz >> obs.snr_dbhz >> obs.elevation_deg >> obs.azimuth_deg)) {
            continue;
        }

        if (epochs.empty() || std::abs(epochs.back().t - t) > 1e-9) {
            GnssEpoch epoch;
            epoch.t = t;
            epochs.push_back(std::move(epoch));
        }
        epochs.back().observations.push_back(obs);
    }
    ensure_nonempty(epochs, path);
    return epochs;
}

std::vector<CameraFrame> load_camera_frames(const std::filesystem::path& timestamp_path,
                                            const std::filesystem::path& image_dir) {
    std::ifstream input(timestamp_path);
    if (!input) {
        throw std::runtime_error("Failed to open " + timestamp_path.string());
    }

    std::vector<CameraFrame> frames;
    std::string line;
    while (std::getline(input, line)) {
        if (!is_data_line(line)) {
            continue;
        }

        std::istringstream iss(line);
        CameraFrame frame;
        if (!(iss >> frame.frame_id >> frame.t)) {
            continue;
        }
        frame.image_path = image_dir / (frame.frame_id + ".png");
        if (std::filesystem::exists(frame.image_path)) {
            frames.push_back(std::move(frame));
        }
    }
    ensure_nonempty(frames, timestamp_path);
    return frames;
}

std::vector<GnssPositionFix> derive_gnss_position_fixes(
    const std::vector<GnssEpoch>& epochs,
    const Vec3& anchor_position_ned,
    GnssSppDiagnostics* diagnostics) {
    if (epochs.empty()) {
        return {};
    }

    constexpr double kElevationMaskDeg = 10.0;
    constexpr double kReferenceSnrDbHz = 45.0;
    constexpr double kMinSnrWeight = 0.1;
    constexpr double kMaxSnrWeight = 10.0;
    constexpr int kRobustIterations = 3;

    // [G3_EXPERIMENTAL] Reproducible degradation of the synthetic GNSS stream.
    // Disabled unless OFFLINE_FUSION_G3=1; raw CSV files remain untouched.
    const bool g3 = std::getenv("OFFLINE_FUSION_G3") != nullptr;
    const bool g4 = std::getenv("OFFLINE_FUSION_G4") != nullptr;
    std::mt19937 rng(20260911u);
    const double rho_sigma = g4 ? 3.0 : 2.0;
    const double sat_sigma = g4 ? 2.0 : 1.0;
    const double mp_sigma = g4 ? 3.0 : 2.0;
    std::normal_distribution<double> white(0.0, rho_sigma);
    std::normal_distribution<double> sat_noise(0.0, sat_sigma);
    std::unordered_map<int,double> multipath;
    const auto& reference_epoch = epochs.front();
    std::unordered_map<int, Vec3> virtual_satellites;
    for (const auto& observation : reference_epoch.observations) {
        if (!std::isfinite(observation.pseudorange_m) ||
            observation.pseudorange_m <= 0.0 ||
            observation.elevation_deg < kElevationMaskDeg) {
            continue;
        }
        const Vec3 los = line_of_sight_ned(
            observation.elevation_deg, observation.azimuth_deg);
        const double rho = observation.pseudorange_m + ((g3 || g4) ? white(rng) : 0.0);
        Vec3 sat = anchor_position_ned + los * rho;
        if (g3 || g4) {
            sat.x += sat_noise(rng);
            sat.y += sat_noise(rng);
            sat.z += sat_noise(rng);
        }
        virtual_satellites[observation.prn] = sat;
    }
    if (virtual_satellites.size() < 4) {
        throw std::runtime_error(
            "SPP-like GNSS initialization requires at least four virtual satellites");
    }

    struct WeightedObservation {
        const GnssObservation* observation = nullptr;
        Vec3 satellite_position;
        Vec3 los;
        double normalized_weight = 1.0;
        double robust_weight = 1.0;
        double pseudorange_m = 0.0;
    };

    auto observation_weight = [=](const GnssObservation& observation) {
        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        const double sin_elevation = std::sin(observation.elevation_deg * kDegToRad);
        const double elevation_weight = sin_elevation * sin_elevation;
        const double snr_weight = std::max(
            kMinSnrWeight,
            std::min(kMaxSnrWeight,
                     std::pow(10.0, (observation.snr_dbhz - kReferenceSnrDbHz) / 10.0)));
        return elevation_weight * snr_weight;
    };

    auto median = [](std::vector<double> values) {
        if (values.empty()) {
            return 0.0;
        }
        const std::size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        double result = values[middle];
        if (values.size() % 2 == 0) {
            const auto lower = std::max_element(values.begin(), values.begin() + middle);
            result = 0.5 * (result + *lower);
        }
        return result;
    };

    std::vector<GnssPositionFix> fixes;
    fixes.reserve(epochs.size());
    double postfit_rms_sum = 0.0;
    double consistency_squared_sum = 0.0;
    double consistency_max = 0.0;
    std::size_t consistency_count = 0;
    double global_min_weight = std::numeric_limits<double>::infinity();
    double global_max_weight = 0.0;

    for (const auto& epoch : epochs) {
        std::vector<WeightedObservation> observations;
        observations.reserve(epoch.observations.size());
        double weight_sum = 0.0;
        for (const auto& observation : epoch.observations) {
            const auto satellite = virtual_satellites.find(observation.prn);
            if (satellite == virtual_satellites.end() ||
                !std::isfinite(observation.pseudorange_m) ||
                observation.pseudorange_m <= 0.0 ||
                !std::isfinite(observation.snr_dbhz) ||
                !std::isfinite(observation.elevation_deg) ||
                !std::isfinite(observation.azimuth_deg) ||
                observation.elevation_deg < kElevationMaskDeg) {
                continue;
            }
            const double weight = observation_weight(observation);
            double rho = observation.pseudorange_m;
            if (g3 || g4) {
                double &b = multipath[observation.prn];
                b = 0.98*b + std::sqrt(1.0-0.98*0.98)*mp_sigma*white(rng)/rho_sigma;
                rho += white(rng) + b;
            }
            observations.push_back({
                &observation,
                satellite->second,
                line_of_sight_ned(observation.elevation_deg, observation.azimuth_deg),
                weight,
                1.0,
                rho,
            });
            weight_sum += weight;
        }

        if (observations.size() < 4 || weight_sum <= 0.0) {
            continue;
        }

        const double mean_weight = weight_sum / static_cast<double>(observations.size());
        for (auto& observation : observations) {
            observation.normalized_weight /= mean_weight;
        }

        auto solve_epoch = [&observations](
                               std::array<std::array<double, 4>, 4>* final_normal) {
            std::array<std::array<double, 4>, 4> normal{};
            std::array<double, 4> rhs{};
            for (const auto& observation : observations) {
                const double weight =
                    observation.normalized_weight * observation.robust_weight;
                const Vec3 measured_endpoint =
                    observation.satellite_position -
                    observation.los * observation.pseudorange_m;
                const double values[3] = {
                    measured_endpoint.x, measured_endpoint.y, measured_endpoint.z};
                const double los_values[3] = {
                    observation.los.x, observation.los.y, observation.los.z};
                for (std::size_t dim = 0; dim < 3; ++dim) {
                    double row[4] = {0.0, 0.0, 0.0, -los_values[dim]};
                    row[dim] = 1.0;
                    for (std::size_t i = 0; i < 4; ++i) {
                        rhs[i] += weight * row[i] * values[dim];
                        for (std::size_t j = 0; j < 4; ++j) {
                            normal[i][j] += weight * row[i] * row[j];
                        }
                    }
                }
            }
            if (final_normal != nullptr) {
                *final_normal = normal;
            }
            return solve_4x4(normal, rhs);
        };

        std::array<double, 4> solution{};
        std::array<std::array<double, 4>, 4> final_normal{};
        try {
            for (int iteration = 0; iteration < kRobustIterations; ++iteration) {
                solution = solve_epoch(nullptr);
                const Vec3 position{solution[0], solution[1], solution[2]};
                std::vector<double> residual_norms;
                residual_norms.reserve(observations.size());
                for (const auto& observation : observations) {
                    const Vec3 predicted_satellite =
                        position + observation.los *
                            (observation.pseudorange_m - solution[3]);
                    residual_norms.push_back(
                        norm(observation.satellite_position - predicted_satellite));
                }
                const double robust_scale =
                    std::max(0.5, 1.4826 * median(residual_norms));
                const double huber_threshold = 1.5 * robust_scale;
                for (std::size_t i = 0; i < observations.size(); ++i) {
                    observations[i].robust_weight = residual_norms[i] <= huber_threshold
                                                        ? 1.0
                                                        : huber_threshold / residual_norms[i];
                }
            }
            solution = solve_epoch(&final_normal);
        } catch (const std::runtime_error&) {
            continue;
        }

        GnssPositionFix fix;
        fix.t = epoch.t;
        fix.position_ned = {solution[0], solution[1], solution[2]};
        fix.clock_bias_m = solution[3];
        fix.observations = observations.size();
        fix.valid = true;
        fix.min_normalized_weight = std::numeric_limits<double>::infinity();
        double residual_squared_sum = 0.0;
        double weighted_residual_squared_sum = 0.0;
        for (const auto& observation : observations) {
            const Vec3 predicted_satellite =
                fix.position_ned + observation.los *
                            (observation.pseudorange_m - fix.clock_bias_m);
            const double residual_norm =
                norm(observation.satellite_position - predicted_satellite);
            residual_squared_sum += residual_norm * residual_norm;
            weighted_residual_squared_sum +=
                observation.normalized_weight * observation.robust_weight *
                residual_norm * residual_norm;
            consistency_squared_sum += residual_norm * residual_norm;
            consistency_max = std::max(consistency_max, residual_norm);
            ++consistency_count;
            fix.min_normalized_weight =
                std::min(fix.min_normalized_weight, observation.normalized_weight);
            fix.max_normalized_weight =
                std::max(fix.max_normalized_weight, observation.normalized_weight);
        }
        fix.postfit_rms_m = std::sqrt(
            residual_squared_sum / static_cast<double>(observations.size()));
        const std::size_t dof = std::max<std::size_t>(1, 3 * observations.size() - 4);
        const double variance_scale =
            weighted_residual_squared_sum / static_cast<double>(dof);
        double* std_components[3] = {
            &fix.position_std_ned.x,
            &fix.position_std_ned.y,
            &fix.position_std_ned.z,
        };
        for (std::size_t dim = 0; dim < 3; ++dim) {
            std::array<double, 4> unit{};
            unit[dim] = 1.0;
            const auto inverse_column = solve_4x4(final_normal, unit);
            const double variance = std::max(0.0, variance_scale * inverse_column[dim]);
            *std_components[dim] = std::max(0.05, std::min(10.0, std::sqrt(variance)));
        }
        postfit_rms_sum += fix.postfit_rms_m;
        global_min_weight = std::min(global_min_weight, fix.min_normalized_weight);
        global_max_weight = std::max(global_max_weight, fix.max_normalized_weight);
        fixes.push_back(fix);
    }

    ensure_nonempty(fixes, "gnss_raw-derived position fixes");

    for (std::size_t i = 0; i < fixes.size(); ++i) {
        const std::size_t left = i > 2 ? i - 2 : 0;
        const std::size_t right = std::min(fixes.size() - 1, i + 2);
        const double dt = fixes[right].t - fixes[left].t;
        if (dt > 0.0) {
            fixes[i].velocity_ned =
                (fixes[right].position_ned - fixes[left].position_ned) / dt;
        }
    }

    if (diagnostics != nullptr) {
        diagnostics->epochs_total = epochs.size();
        diagnostics->epochs_solved = fixes.size();
        diagnostics->epochs_rejected = epochs.size() - fixes.size();
        diagnostics->virtual_satellites = virtual_satellites.size();
        for (const auto& fix : fixes) {
            diagnostics->observations_used += fix.observations;
        }
        diagnostics->mean_satellites =
            static_cast<double>(diagnostics->observations_used) /
            static_cast<double>(fixes.size());
        diagnostics->mean_postfit_rms_m =
            postfit_rms_sum / static_cast<double>(fixes.size());
        diagnostics->max_postfit_rms_m = std::max_element(
            fixes.begin(), fixes.end(), [](const GnssPositionFix& a, const GnssPositionFix& b) {
                return a.postfit_rms_m < b.postfit_rms_m;
            })->postfit_rms_m;
        diagnostics->min_normalized_weight = global_min_weight;
        diagnostics->max_normalized_weight = global_max_weight;
        if (consistency_count > 0) {
            diagnostics->fixed_satellite_consistency_rms_m = std::sqrt(
                consistency_squared_sum / static_cast<double>(consistency_count));
            diagnostics->fixed_satellite_consistency_max_m = consistency_max;
        }
        diagnostics->fixed_satellite_model_consistent =
            diagnostics->fixed_satellite_consistency_rms_m <= 10.0 &&
            diagnostics->fixed_satellite_consistency_max_m <= 30.0;
    }
    return fixes;
}

namespace {

std::vector<GroundTruthSample> load_structured_groundtruth(
    const std::filesystem::path& path,
    std::int64_t origin_ns) {
    CsvTableReader reader(path);
    std::vector<GroundTruthSample> samples;
    while (reader.next()) {
        GroundTruthSample sample;
        sample.t = structured_time_seconds(reader, "timestamp_ns", origin_ns);
        sample.phase = reader.get("phase");
        sample.position_ned = {
            std::stod(reader.get("true_n_m")),
            std::stod(reader.get("true_e_m")),
            std::stod(reader.get("true_d_m")),
        };
        sample.velocity_ned = {
            std::stod(reader.get("true_vn_mps")),
            std::stod(reader.get("true_ve_mps")),
            std::stod(reader.get("true_vd_mps")),
        };
        sample.has_velocity = true;
        samples.push_back(sample);
    }
    ensure_nonempty(samples, path);
    return samples;
}

std::vector<ImuSample> load_structured_imu(const std::filesystem::path& path,
                                           std::int64_t origin_ns) {
    CsvTableReader reader(path);
    std::vector<ImuSample> samples;
    while (reader.next()) {
        ImuSample sample;
        sample.t = structured_time_seconds(reader, "sim_timestamp_ns", origin_ns);
        sample.phase = reader.get("phase");
        sample.accel = {
            std::stod(reader.get("ax_mps2")),
            std::stod(reader.get("ay_mps2")),
            std::stod(reader.get("az_mps2")),
        };
        sample.gyro = {
            std::stod(reader.get("gx_radps")),
            std::stod(reader.get("gy_radps")),
            std::stod(reader.get("gz_radps")),
        };
        samples.push_back(sample);
    }
    ensure_nonempty(samples, path);
    return samples;
}

std::vector<GnssEpoch> load_structured_gnss_raw(const std::filesystem::path& path,
                                                std::int64_t origin_ns) {
    CsvTableReader reader(path);
    std::vector<GnssEpoch> epochs;
    while (reader.next()) {
        const double t = structured_time_seconds(reader, "sim_timestamp_ns", origin_ns);
        if (epochs.empty() || std::abs(epochs.back().t - t) > 1e-9) {
            GnssEpoch epoch;
            epoch.t = t;
            epoch.phase = reader.get("phase");
            epochs.push_back(std::move(epoch));
        }
        GnssObservation observation;
        observation.prn = std::stoi(reader.get("prn"));
        observation.pseudorange_m = std::stod(reader.get("pseudorange_m"));
        observation.carrier_phase_cyc = std::stod(reader.get("carrier_phase_cycles"));
        observation.doppler_hz = std::stod(reader.get("doppler_hz"));
        observation.snr_dbhz = std::stod(reader.get("snr_dbhz"));
        observation.elevation_deg = std::stod(reader.get("elevation_deg"));
        observation.azimuth_deg = std::stod(reader.get("azimuth_deg"));
        epochs.back().observations.push_back(observation);
    }
    ensure_nonempty(epochs, path);
    return epochs;
}

std::vector<GnssFixMeasurement> load_structured_gnss_fixes(
    const std::filesystem::path& path,
    std::int64_t origin_ns,
    const GeodeticOrigin& geodetic_origin) {
    CsvTableReader reader(path);
    std::vector<GnssFixMeasurement> fixes;
    while (reader.next()) {
        GnssFixMeasurement fix;
        fix.t = structured_time_seconds(reader, "sim_timestamp_ns", origin_ns);
        fix.phase = reader.get("phase");
        fix.valid = parse_bool(reader.get("is_valid"));
        fix.fix_type = std::stoi(reader.get("fix_type"));
        fix.latitude_deg = std::stod(reader.get("latitude_deg"));
        fix.longitude_deg = std::stod(reader.get("longitude_deg"));
        fix.altitude_m = std::stod(reader.get("altitude_m"));
        fix.horizontal_std_m = std::stod(reader.get("eph_m"));
        fix.vertical_std_m = std::stod(reader.get("epv_m"));
        fix.position_ned = geodetic_to_ned(
            fix.latitude_deg, fix.longitude_deg, fix.altitude_m, geodetic_origin);
        fix.velocity_ned = {
            std::stod(reader.get("velocity_n_mps")),
            std::stod(reader.get("velocity_e_mps")),
            std::stod(reader.get("velocity_d_mps")),
        };
        fixes.push_back(fix);
    }
    ensure_nonempty(fixes, path);
    return fixes;
}

std::vector<CameraFrame> load_structured_camera_frames(
    const std::filesystem::path& timestamp_path,
    const std::filesystem::path& image_dir,
    std::int64_t origin_ns) {
    CsvTableReader reader(timestamp_path);
    std::vector<CameraFrame> frames;
    while (reader.next()) {
        CameraFrame frame;
        const std::string filename = reader.get("filename");
        frame.frame_id = std::filesystem::path(filename).stem().string();
        frame.t = relative_seconds(std::stoll(reader.get("sim_timestamp_ns")), origin_ns);
        frame.image_path = image_dir / filename;
        if (!std::filesystem::is_regular_file(frame.image_path)) {
            throw std::runtime_error("Missing camera image " + frame.image_path.string());
        }
        frames.push_back(std::move(frame));
    }
    ensure_nonempty(frames, timestamp_path);
    return frames;
}

std::vector<LidarFrame> load_structured_lidar_frames(
    const std::filesystem::path& timestamp_path,
    const std::filesystem::path& point_dir,
    std::int64_t origin_ns) {
    CsvTableReader reader(timestamp_path);
    std::vector<LidarFrame> frames;
    while (reader.next()) {
        LidarFrame frame;
        const std::string filename = reader.get("filename");
        frame.frame_id = std::filesystem::path(filename).stem().string();
        frame.t = relative_seconds(std::stoll(reader.get("sim_timestamp_ns")), origin_ns);
        frame.point_count = static_cast<std::size_t>(std::stoull(reader.get("point_count")));
        frame.point_path = point_dir / filename;
        if (!std::filesystem::is_regular_file(frame.point_path)) {
            throw std::runtime_error("Missing LiDAR point file " + frame.point_path.string());
        }
        const auto expected_bytes =
            static_cast<std::uintmax_t>(frame.point_count * sizeof(LidarPoint));
        const auto actual_bytes = std::filesystem::file_size(frame.point_path);
        if (actual_bytes != expected_bytes) {
            throw std::runtime_error("Unexpected LiDAR file size for " + frame.point_path.string());
        }
        frames.push_back(std::move(frame));
    }
    ensure_nonempty(frames, timestamp_path);
    return frames;
}

std::vector<FiveGEpoch> load_structured_fiveg(const std::filesystem::path& path,
                                              std::int64_t origin_ns) {
    CsvTableReader reader(path);
    std::vector<FiveGEpoch> epochs;
    while (reader.next()) {
        const double t = structured_time_seconds(reader, "timestamp_ns", origin_ns);
        if (epochs.empty() || std::abs(epochs.back().t - t) > 1e-9) {
            FiveGEpoch epoch;
            epoch.t = t;
            epoch.phase = reader.get("phase");
            epochs.push_back(std::move(epoch));
        }
        FiveGObservation observation;
        observation.gnb_id = reader.get("gnb_id");
        observation.gnb_position_ned = {
            std::stod(reader.get("gnb_n_m")),
            std::stod(reader.get("gnb_e_m")),
            std::stod(reader.get("gnb_d_m")),
        };
        observation.prs_rsrp_dbm = std::stod(reader.get("prs_rsrp_dbm"));
        observation.is_los = parse_bool(reader.get("is_los"));
        observation.range_std_m = std::stod(reader.get("range_sigma_m"));
        observation.measured_range_m = std::stod(reader.get("measured_range_m"));
        observation.measured_rtt_s = std::stod(reader.get("measured_rtt_s"));
        observation.valid = parse_bool(reader.get("measurement_valid")) &&
                            std::isfinite(observation.measured_range_m) &&
                            observation.range_std_m > 0.0;
        epochs.back().observations.push_back(std::move(observation));
    }
    ensure_nonempty(epochs, path);
    return epochs;
}

Dataset load_structured_dataset(const std::filesystem::path& data_dir) {
    const auto truth_path = data_dir / "ground_truth" / "truth.csv";
    const auto imu_path = data_dir / "sensors" / "imu" / "imu.csv";
    const auto gnss_fix_path = data_dir / "sensors" / "gnss" / "gps_fix.csv";
    const auto gnss_raw_path = data_dir / "sensors" / "gnss" / "raw_observations.csv";
    const auto camera_timestamp_path = data_dir / "sensors" / "camera" / "timestamps.csv";
    const auto lidar_timestamp_path = data_dir / "sensors" / "lidar" / "timestamps.csv";
    const auto fiveg_path = data_dir / "sensors" / "5g_a" / "raw_observations.csv";

    const std::array<std::int64_t, 6> first_timestamps = {
        read_first_timestamp(imu_path, "sim_timestamp_ns"),
        read_first_timestamp(gnss_fix_path, "sim_timestamp_ns"),
        read_first_timestamp(gnss_raw_path, "sim_timestamp_ns"),
        read_first_timestamp(camera_timestamp_path, "sim_timestamp_ns"),
        read_first_timestamp(lidar_timestamp_path, "sim_timestamp_ns"),
        read_first_timestamp(fiveg_path, "timestamp_ns"),
    };

    std::int64_t elapsed_time_origin_ns = 0;
    const bool has_elapsed_time_origin = read_elapsed_time_origin(
        imu_path, "sim_timestamp_ns", &elapsed_time_origin_ns);

    Dataset dataset;
    dataset.format = DatasetFormat::StructuredExperiment;
    dataset.time_origin_ns = has_elapsed_time_origin
        ? elapsed_time_origin_ns
        : *std::min_element(first_timestamps.begin(), first_timestamps.end());
    dataset.geodetic_origin = {
        47.64146799999743,
        -122.140165,
        122.000091552734,
        true,
    };
    // Truth is optional evaluation data; the estimator's time origin and GNSS
    // frame can be established from recorded sensor measurements alone.
    if (std::filesystem::is_regular_file(truth_path)) {
        dataset.groundtruth = load_structured_groundtruth(truth_path, dataset.time_origin_ns);
    }
    dataset.imu = load_structured_imu(imu_path, dataset.time_origin_ns);
    dataset.gnss_epochs = load_structured_gnss_raw(gnss_raw_path, dataset.time_origin_ns);
    dataset.gnss_fixes = load_structured_gnss_fixes(
        gnss_fix_path, dataset.time_origin_ns, dataset.geodetic_origin);
    dataset.camera_frames = load_structured_camera_frames(
        camera_timestamp_path,
        data_dir / "sensors" / "camera" / "images",
        dataset.time_origin_ns);
    dataset.lidar_frames = load_structured_lidar_frames(
        lidar_timestamp_path,
        data_dir / "sensors" / "lidar" / "points",
        dataset.time_origin_ns);
    dataset.fiveg_epochs = load_structured_fiveg(fiveg_path, dataset.time_origin_ns);

    // The raw stream stores pseudorange and LOS but no satellite ephemerides.
    // Its virtual-satellite frame therefore needs an observed position anchor.
    // Use the valid GNSS fix at the reference raw epoch, never ground truth.
    const auto reference_fix = std::find_if(
        dataset.gnss_fixes.begin(), dataset.gnss_fixes.end(),
        [&](const GnssFixMeasurement& fix) {
            return fix.valid && std::abs(fix.t - dataset.gnss_epochs.front().t) <= 0.05 &&
                   std::isfinite(fix.position_ned.x) &&
                   std::isfinite(fix.position_ned.y) &&
                   std::isfinite(fix.position_ned.z);
        });
    if (reference_fix == dataset.gnss_fixes.end()) {
        throw std::runtime_error(
            "No valid same-epoch GNSS fix for virtual-satellite frame anchor");
    }
    const Vec3 anchor_position = reference_fix->position_ned;
    dataset.gnss_position_fixes =
        derive_gnss_position_fixes(
            dataset.gnss_epochs, anchor_position, &dataset.gnss_spp_diagnostics);
    return dataset;
}

Dataset load_legacy_dataset(const std::filesystem::path& data_dir) {
    Dataset dataset;
    dataset.format = DatasetFormat::LegacyText;
    dataset.imu = load_imu(data_dir / "imu.txt");
    dataset.groundtruth = load_groundtruth(data_dir / "groundtruth.txt");
    dataset.gnss_epochs = load_gnss_raw(data_dir / "gnss_raw.txt");
    const Vec3 anchor_position =
        interpolate_groundtruth_position(dataset.groundtruth, dataset.gnss_epochs.front().t);
    dataset.gnss_position_fixes =
        derive_gnss_position_fixes(
            dataset.gnss_epochs, anchor_position, &dataset.gnss_spp_diagnostics);
    const std::filesystem::path camera_timestamps = data_dir / "camera_timestamps.txt";
    const std::filesystem::path image_dir = data_dir / "image";
    if (std::filesystem::exists(camera_timestamps) && std::filesystem::is_directory(image_dir)) {
        dataset.camera_frames = load_camera_frames(camera_timestamps, image_dir);
    }
    return dataset;
}

}  // namespace

Dataset load_dataset(const std::filesystem::path& data_dir) {
    if (std::filesystem::is_regular_file(data_dir / "sensors" / "imu" / "imu.csv") &&
        std::filesystem::is_regular_file(
            data_dir / "sensors" / "gnss" / "raw_observations.csv")) {
        return load_structured_dataset(data_dir);
    }
    if (std::filesystem::is_regular_file(data_dir / "imu.txt")) {
        return load_legacy_dataset(data_dir);
    }
    throw std::runtime_error("Unrecognized dataset layout at " + data_dir.string());
}

const char* dataset_format_name(DatasetFormat format) {
    switch (format) {
        case DatasetFormat::LegacyText:
            return "legacy_text";
        case DatasetFormat::StructuredExperiment:
            return "structured_experiment";
    }
    return "unknown";
}

std::vector<LidarPoint> load_lidar_points(const LidarFrame& frame) {
    std::ifstream input(frame.point_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Failed to open " + frame.point_path.string());
    }
    std::vector<LidarPoint> points(frame.point_count);
    input.read(reinterpret_cast<char*>(points.data()),
               static_cast<std::streamsize>(points.size() * sizeof(LidarPoint)));
    if (!input || input.gcount() !=
                      static_cast<std::streamsize>(points.size() * sizeof(LidarPoint))) {
        throw std::runtime_error("Failed to read complete LiDAR frame " +
                                 frame.point_path.string());
    }
    return points;
}

Vec3 interpolate_groundtruth_position(const std::vector<GroundTruthSample>& gt, double t) {
    if (gt.empty()) {
        throw std::runtime_error("Cannot interpolate empty groundtruth vector");
    }
    if (t <= gt.front().t) {
        return gt.front().position_ned;
    }
    if (t >= gt.back().t) {
        return gt.back().position_ned;
    }

    const auto it = std::lower_bound(
        gt.begin(), gt.end(), t,
        [](const GroundTruthSample& sample, double value) { return sample.t < value; });
    const auto& b = *it;
    const auto& a = *(it - 1);
    const double alpha = (t - a.t) / (b.t - a.t);
    return lerp(a.position_ned, b.position_ned, alpha);
}

Quat nearest_groundtruth_orientation(const std::vector<GroundTruthSample>& gt, double t) {
    if (gt.empty()) {
        throw std::runtime_error("Cannot query empty groundtruth vector");
    }
    const auto it = std::lower_bound(
        gt.begin(), gt.end(), t,
        [](const GroundTruthSample& sample, double value) { return sample.t < value; });
    if (it == gt.begin()) {
        return it->q_body_to_ned;
    }
    if (it == gt.end()) {
        return gt.back().q_body_to_ned;
    }
    const auto& prev = *(it - 1);
    return (t - prev.t <= it->t - t) ? prev.q_body_to_ned : it->q_body_to_ned;
}

Vec3 estimate_groundtruth_velocity(const std::vector<GroundTruthSample>& gt, double t) {
    if (!gt.empty() && gt.front().has_velocity) {
        if (t <= gt.front().t) {
            return gt.front().velocity_ned;
        }
        if (t >= gt.back().t) {
            return gt.back().velocity_ned;
        }
        const auto it = std::lower_bound(
            gt.begin(), gt.end(), t,
            [](const GroundTruthSample& sample, double value) { return sample.t < value; });
        const auto& b = *it;
        const auto& a = *(it - 1);
        const double alpha = (t - a.t) / (b.t - a.t);
        return lerp(a.velocity_ned, b.velocity_ned, alpha);
    }
    const double dt = 0.1;
    const Vec3 p0 = interpolate_groundtruth_position(gt, t - dt);
    const Vec3 p1 = interpolate_groundtruth_position(gt, t + dt);
    return (p1 - p0) / (2.0 * dt);
}

}  // namespace offline_fusion
