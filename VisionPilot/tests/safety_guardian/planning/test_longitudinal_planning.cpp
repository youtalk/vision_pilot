// Headless assertion tests for the longitudinal planner's curvature preview
// and for the spatial step shared with the lateral MPC.
//
// Exit status is the number of failures, so this runs unattended.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <planning/lateral_planning.hpp>
#include <planning/longitudinal_planning.hpp>
#include <planning/planning.hpp>

namespace {

int failures = 0;

void check(const bool ok, const std::string& what)
{
    std::printf("%s  %s\n", ok ? "[ ok ]" : "[FAIL]", what.c_str());
    if (!ok) ++failures;
}

void check_near(const double got, const double want, const double tol, const std::string& what)
{
    const bool ok = std::abs(got - want) <= tol;
    std::printf("%s  %s (got %.4f, want %.4f +/- %.4f)\n",
                ok ? "[ ok ]" : "[FAIL]", what.c_str(), got, want, tol);
    if (!ok) ++failures;
}

LongitudinalPlanner::Config make_config(const double speed_limit)
{
    LongitudinalPlanner::Config c;
    c.speed_limit = speed_limit;
    return c;
}

// Curvature of the Town04 curve that put the ego in the lake, and the tightest
// curvature observed on that spawn.
constexpr double KAPPA_CURVE   = 0.0088;   // R ~ 116 m
constexpr double KAPPA_TIGHTEST = 0.0128;

// Build a preview of a road that is straight until s_curve, then constant.
void build_road_preview(const double v, const double s_to_curve,
                        const LongitudinalPlanner::Config& cfg,
                        Eigen::VectorXd& kappa_preview, Eigen::VectorXd& s_preview)
{
    const int    M    = static_cast<int>(N);
    const double span = std::max(cfg.preview_min_distance, v * cfg.preview_time);
    kappa_preview.resize(M);
    s_preview.resize(M);
    for (int i = 0; i < M; ++i) {
        const double s = span * static_cast<double>(i) / static_cast<double>(M - 1);
        s_preview[i]     = s;
        kappa_preview[i] = (s >= s_to_curve) ? KAPPA_CURVE : 0.0;
    }
}

// ── Fix 2: curvature preview ────────────────────────────────────────────────

void test_a_straight_road_places_no_restriction()
{
    const auto cfg = make_config(33.3);
    LongitudinalPlanner planner(cfg);

    Eigen::VectorXd kappa = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd s     = Eigen::VectorXd::LinSpaced(N, 0.0, 90.0);

    check(std::isinf(planner.curve_acceleration(kappa, s, 29.18)),
          "a straight road demands no deceleration at all");
}

void test_holding_the_curve_speed_in_a_curve_demands_nothing()
{
    const auto cfg = make_config(33.3);
    LongitudinalPlanner planner(cfg);

    Eigen::VectorXd kappa = Eigen::VectorXd::Constant(N, KAPPA_CURVE);
    Eigen::VectorXd s     = Eigen::VectorXd::LinSpaced(N, 0.0, 90.0);

    const double v_curve = std::sqrt(cfg.mu * cfg.g / KAPPA_CURVE);
    check_near(planner.curve_acceleration(kappa, s, v_curve), 0.0, 1e-6,
               "at the curve's own speed the curve demands zero acceleration");
}

void test_being_too_fast_in_a_curve_demands_comfortable_braking()
{
    const auto cfg = make_config(33.3);
    LongitudinalPlanner planner(cfg);

    Eigen::VectorXd kappa = Eigen::VectorXd::Constant(N, KAPPA_CURVE);
    Eigen::VectorXd s     = Eigen::VectorXd::LinSpaced(N, 0.0, 90.0);

    check_near(planner.curve_acceleration(kappa, s, 29.18), -cfg.b, 1e-9,
               "already in the curve and too fast floors at comfortable braking");
}

void test_the_demand_grows_as_the_curve_approaches()
{
    const auto cfg = make_config(33.3);
    LongitudinalPlanner planner(cfg);

    double previous = 1e9;
    bool   monotonic = true;
    for (const double s_to_curve : {80.0, 60.0, 40.0, 20.0}) {
        Eigen::VectorXd kappa, s;
        build_road_preview(29.18, s_to_curve, cfg, kappa, s);
        const double a = planner.curve_acceleration(kappa, s, 29.18);
        if (a > previous) monotonic = false;
        previous = a;
    }
    check(monotonic, "the deceleration demand grows as the curve nears");
}

void test_the_approach_arrives_at_the_curve_speed_without_a_spike()
{
    // The failing scenario: 29.18 m/s (65.3 mph) toward a curve 120 m ahead.
    // Without preview the planner emitted -20.613 m/s^2 on arrival, which the
    // CARLA bridge turned into a 0 m/s target. -8.0 is the bridge's clamp.
    const auto cfg = make_config(33.3);
    LongitudinalPlanner planner(cfg);

    double v            = 29.18;
    double s_to_curve   = 120.0;
    double min_accel    = 0.0;
    double decel_onset  = -1.0;    // distance to the curve when braking began
    const double step   = 0.05;    // s

    while (s_to_curve > 0.0 && v > 0.1) {
        Eigen::VectorXd kappa, s;
        build_road_preview(v, s_to_curve, cfg, kappa, s);

        const double a_idm   = planner.compute_acceleration(v, false, cfg.speed_limit, 9999.0);
        const double a_curve = planner.curve_acceleration(kappa, s, v);
        const double a       = std::min(a_idm, a_curve);

        if (a < -0.1 && decel_onset < 0.0) decel_onset = s_to_curve;
        if (a < min_accel) min_accel = a;

        v          = std::max(0.0, v + a * step);
        s_to_curve -= v * step;
    }

    const double curve_limit = std::sqrt(cfg.mu * cfg.g / KAPPA_CURVE);

    check(min_accel > -8.0,
          "the approach never reaches the bridge's -8.0 clamp (was -20.613)");
    check(min_accel >= -cfg.b - 1e-9,
          "the approach stays within comfortable braking");
    check(decel_onset > 30.0,
          "braking starts more than 30 m before the curve");
    check(v <= curve_limit * 1.05,
          "the ego arrives at the curve's speed limit");
    std::printf("       approach: min accel %.3f m/s^2, braking began %.1f m out, "
                "entry speed %.2f m/s (curve limit %.2f)\n",
                min_accel, decel_onset, v, curve_limit);
}

void test_mu_permits_a_realistic_speed_through_the_tightest_town04_curve()
{
    const auto cfg = make_config(33.3);
    check_near(std::sqrt(cfg.mu * cfg.g / KAPPA_TIGHTEST), 16.4, 0.3,
               "tightest Town04 curve permits ~16.4 m/s (37 mph)");
}

// ── Curve braking must not read as a collision ──────────────────────────────

void test_braking_for_a_curve_raises_no_collision_warning()
{
    // FCW fires on -5.0 <= a <= -3.0 and curve braking floors at exactly -b
    // (-3.0), so without a guard every high-speed curve entry would report a
    // forward collision with nothing in front of the ego.
    Planner planner(33.3, 1.4);

    bool saw_collision_warning = false;
    double v = 29.18;

    // Drive into a tightening curve with no lead vehicle.
    for (int i = 0; i < 40; ++i) {
        const Plan plan = planner.compute_plan(0.0, 0.0, KAPPA_CURVE, v, false, 33.3, 9999.0);
        for (const Warning w : plan.warnings)
            if (w == Warning::FCW || w == Warning::AEB) saw_collision_warning = true;
        v = std::max(1.0, v + plan.acceleration * 0.05);
    }

    check(!saw_collision_warning,
          "braking for a curve raises neither FCW nor AEB");
}

// ── Fix 3: one spatial step ─────────────────────────────────────────────────

void test_the_horizon_step_matches_the_mpc_integration_step()
{
    check_near(horizon_step(4.0), 0.30, 1e-9,
               "horizon step floors at 0.30 m for a slow ego");
    check_near(horizon_step(29.18), 29.18 / static_cast<double>(N), 1e-9,
               "horizon step spans ~1 s of travel at highway speed");
}

void test_the_curvature_schedule_spans_the_mpc_horizon()
{
    // A constant-curvature road: every sample along the horizon should read
    // back the same curvature, at whatever step the MPC is integrating.
    const double v  = 29.18;
    const double ds = horizon_step(v);

    const Eigen::VectorXd schedule =
        build_kappa_schedule(0.0 /* epsi */, KAPPA_CURVE, 0.0 /* dkappa_ds */, ds, 0.175);

    check(schedule.size() == static_cast<int>(N),
          "the schedule has one sample per MPC step");
    check_near(static_cast<double>(N - 1) * ds, 27.7, 0.5,
               "the schedule spans the MPC's own horizon, not 20 x 0.5 m");

    // Sample i must sit at arc-position i*ds on the reference curve, so halving
    // the step and doubling the count must reproduce every original sample.
    const Eigen::VectorXd coarse =
        build_kappa_schedule(0.1, KAPPA_CURVE, 0.001, 2.0, 0.175, 14);
    const Eigen::VectorXd fine =
        build_kappa_schedule(0.1, KAPPA_CURVE, 0.001, 1.0, 0.175, 28);

    double max_diff = 0.0;
    for (int i = 0; i < coarse.size(); ++i)
        max_diff = std::max(max_diff, std::abs(coarse[i] - fine[2 * i]));

    check_near(max_diff, 0.0, 1e-12,
               "samples land at i*ds, so the step alone sets the span");
}

}  // namespace

int main()
{
    test_a_straight_road_places_no_restriction();
    test_holding_the_curve_speed_in_a_curve_demands_nothing();
    test_being_too_fast_in_a_curve_demands_comfortable_braking();
    test_the_demand_grows_as_the_curve_approaches();
    test_the_approach_arrives_at_the_curve_speed_without_a_spike();
    test_mu_permits_a_realistic_speed_through_the_tightest_town04_curve();
    test_braking_for_a_curve_raises_no_collision_warning();
    test_the_horizon_step_matches_the_mpc_integration_step();
    test_the_curvature_schedule_spans_the_mpc_horizon();

    std::printf("\n%d failure(s)\n", failures);
    return failures;
}
