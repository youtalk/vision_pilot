#include <planning/longitudinal_planning.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

double dt = 0.05;

LongitudinalPlanner::LongitudinalPlanner(const Config& config)
    : config_(config) {}

double LongitudinalPlanner::curve_speed_limit(const double kappa) const {
    return std::sqrt(config_.mu * config_.g / std::abs(kappa));   // inf when kappa ~ 0, fine
}

double LongitudinalPlanner::curve_acceleration(const Eigen::VectorXd& kappa_preview,
                                               const Eigen::VectorXd& s_preview,
                                               const double ego_v,
                                               const double s_from) const {
    double demand = std::numeric_limits<double>::infinity();

    for (int i = 0; i < kappa_preview.size(); ++i) {
        const double s = s_preview[i] - s_from;
        if (s < 0.0) continue;   // already behind the ego at this horizon step

        const double v_curve = curve_speed_limit(kappa_preview[i]);
        if (!std::isfinite(v_curve)) continue;   // straight here, no demand

        // Rearranged v² = u² + 2as. Guard s so a curve underfoot gives a large
        // finite demand rather than a division by zero; the floor below then
        // bounds it.
        const double a_req = (v_curve * v_curve - ego_v * ego_v)
                           / (2.0 * std::max(0.05, s));
        demand = std::min(demand, a_req);
    }

    return std::max(-config_.b, demand);   // +inf survives: no curve in view
}

double LongitudinalPlanner::compute_acceleration(double ego_v, bool has_cipo, double cipo_v, double cipo_distance) {

    // Closing speed — negative when ego is slower than lead (gap opening)
    double delta_v = cipo_v; // cipo_v relative CIPO vehicle sppeed

    // Wrap the dynamic term in max(0, …).
    // Without this, when v < lead_v, delta_v < 0 making dynamic_term
    // negative, which shrinks s_star below s0 and collapses the interaction
    // term — IDM would then output full free-road acceleration and overshoot.
    double dynamic_term = (ego_v * delta_v) / (2.0 * std::sqrt(config_.a * config_.b));
    double s_star = config_.s0 + std::max(0.0, ego_v * config_.T + dynamic_term);

    // Raise floor to 0.5 m — matches the gap floor in mpc_test.cpp;
    // prevents (s_star / s)² from becoming catastrophically large.
    double s = std::max(0.5, cipo_distance);

    // Free-road term: positive, drives ego toward speed_limit
    double free_road_term = std::pow(ego_v / config_.speed_limit, config_.delta);

    // Interaction term: only active when a real lead vehicle is present.
    double interaction_term  = has_cipo ? std::pow(s_star / s, 2.0) : 0.0;

    return config_.a * (1.0 - free_road_term - interaction_term);
}
