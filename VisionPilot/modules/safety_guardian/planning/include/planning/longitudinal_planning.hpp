#ifndef VISIONPILOT_LONGITUDIANL_PLANNING_HPP
#define VISIONPILOT_LONGITUDIANL_PLANNING_HPP

#include <Eigen/Core>

extern double dt;   // 50Hz control period — still used by planning.cpp

class LongitudinalPlanner {
public:
    struct Config {
        double speed_limit;  // m/s
        double a     = 1.5;    // max comfortable acceleration  (m/s²)
        double b     = 3.0;    // comfortable deceleration      (m/s²)
        double T     = 1.5;    // desired time-headway          (s)
        double s0    = 2.0;    // minimum gap at standstill     (m)
        double delta = 4.0;    // free-road acceleration exponent

        // Lateral friction budget used to convert road curvature into a speed
        // limit. 0.35 is ~3.4 m/s² of lateral acceleration, a normal comfort
        // limit; the previous 0.2 held the ego to 14.9 m/s (33 mph) on a 116 m
        // radius highway curve.
        double mu = 0.35;
        double g = 9.81;

        // How far ahead the curvature preview looks. The MPC horizon alone
        // (N * ds, ~28 m at highway speed) is too short to shed speed
        // comfortably: braking 29 m/s down to 20 m/s inside 28 m needs
        // -8.6 m/s². Three seconds of travel gives ~88 m at 65 mph.
        double preview_time         = 3.0;    // s
        double preview_min_distance = 30.0;   // m — floor at low speed
    };

    explicit LongitudinalPlanner(const Config& config);

    const Config& config() const { return config_; }

    // Speed at which the ego may take a curve of the given curvature, from the
    // lateral friction budget. Infinite on a straight road (kappa == 0).
    double curve_speed_limit(double kappa) const;

    // The most restrictive acceleration the previewed road curvature demands,
    // from v² = u² + 2as: to be at a point's curve limit on arrival, the ego
    // must already be decelerating at (v_curve² - ego_v²) / 2s.
    //
    //   kappa_preview : signed road curvature at each previewed point (1/m)
    //   s_preview     : distance to each previewed point (m), ascending
    //   ego_v         : ego speed (m/s)
    //   s_from        : distance the ego is predicted to have travelled before
    //                   the preview applies; nearer points are already behind
    //
    // Returns +infinity when no curve is in view, so callers can take
    // min(idm, curve) unconditionally. Floored at -b: a curve is visible well
    // in advance, so comfortable braking always suffices for a real one, and a
    // curvature glitch must not command an emergency stop.
    //
    // A speed limit cannot do this job. IDM's free-road term is a regulator,
    // so against a falling limit it settles at a lag ratio r solving
    // r^delta - (b/a)·r - 1 = 0 — about 1.4 — and the ego would enter the
    // curve 40% too fast however the limit is shaped.
    double curve_acceleration(const Eigen::VectorXd& kappa_preview,
                              const Eigen::VectorXd& s_preview,
                              double ego_v,
                              double s_from = 0.0) const;

    // Returns the IDM acceleration for the current step.
    //
    //   ego_v         : ego speed (m/s)
    //   has_cipo      : CIPO in front
    //   cipo_v        : lead-vehicle speed (m/s); set to speed_limit for free road
    //   cipo_distance : bumper-to-bumper gap (m); use 9999.0 for free road
    double compute_acceleration(double ego_v, bool has_cipo, double cipo_v, double cipo_distance);

private:
    Config config_;
};

#endif //VISIONPILOT_LONGITUDIANL_PLANNING_HPP
