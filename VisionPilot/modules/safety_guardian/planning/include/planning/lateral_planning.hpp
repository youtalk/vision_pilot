#ifndef VISIONPILOT_LATERAL_HPP
#define VISIONPILOT_LATERAL_HPP

#include <algorithm>
#include <vector>
#include <Eigen/Core>

// ── Horizon parameters ───────────────────────────────────────────────────────
extern size_t N;

// The spatial step the MPC integrates on, sized so the N-step horizon spans
// ~1 s of travel. This is the single source of the step: the caller samples the
// curvature reference at it and passes both the schedule and the step into
// compute_steering. Sampling the schedule at some other spacing compresses the
// road's bend relative to the dynamics horizon — a fixed 0.5 m against this
// step's 1.46 m at 65 mph put only 37% of the true distance in front of the
// optimiser, so it under-anticipated the curve.
inline double horizon_step(const double ego_v)
{
    return std::max(0.30, (ego_v * 1.0) / static_cast<double>(N));
}

class LateralPlanner {
public:
    LateralPlanner();
    ~LateralPlanner();

    // Solve the MPC for the steering sequence.
    //
    //   state          = [cte, epsi, kappa_road]   (initial conditions)
    //   ds             = spatial integration step (m), from horizon_step().
    //                    Passed in rather than recomputed here so the caller's
    //                    curvature schedule is sampled on the same grid.
    //   v_schedule     = predicted speed at each horizon step      (N elements)
    //   kappa_schedule = predicted road curvature at each horizon  (N elements)
    //                    step.  Built by the caller from the current curvature
    //                    plus a *clamped* linear preview, so it can never
    //                    exceed the steering-achievable curvature.  This is
    //                    passed in (like v_schedule) rather than reconstructed
    //                    inside the MPC, which keeps the optimiser's
    //                    constraints smooth.
    //
    // Returns [delta_0, delta_0, delta_1, ..., delta_{N-2}]
    std::vector<double> compute_steering(double L,
                                        double ds,
                                        const Eigen::VectorXd& state,
                                        const Eigen::VectorXd& v_schedule,
                                        const Eigen::VectorXd& kappa_schedule);
};

#endif //VISIONPILOT_LATERAL_HPP
