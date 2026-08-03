#include "vehicle.h"
#include <cmath>
#include <algorithm>

std::pair<double, double> axleLoads(const State &s, const CarParams &p, double ax)
{
    double g = 9.81;
    double wb = p.a + p.b;
    double weight = p.mass * g;

    double staticFront = weight * p.b / wb;
    double staticRear = weight * p.a / wb;

    double downforce = p.liftCoeff * s.vx * s.vx;
    double dfFront = downforce * 0.5;
    double dfRear = downforce * 0.5;

    // Longitudinal load transfer: braking (ax < 0) loads the fronts,
    // accelerating (ax > 0) loads the rears.
    double transfer = (p.mass * ax * p.cgHeight) / wb;

    double Fzf = staticFront + dfFront - transfer;
    double Fzr = staticRear + dfRear + transfer;

    // Never let a modeled axle load go negative (would imply the
    // wheel left the ground, which this model can't represent).
    Fzf = std::max(0.0, Fzf);
    Fzr = std::max(0.0, Fzr);

    return {Fzf, Fzr};
}

void step(State &s, const CarParams &p, double throttle,
          double brake, double steer, double dt)
{

    double alphaF = std::atan2(s.vy + p.a * s.r, s.vx) - steer;
    double alphaR = std::atan2(s.vy - p.b * s.r, s.vx);

    // Raw (uncapped) linear tire forces
    double FyfRaw = -p.Cf * alphaF;
    double FyrRaw = -p.Cr * alphaR;

    double drag = p.dragCoeff * s.vx * s.vx;
    double FxDrive = throttle * p.maxDriveF;
    double FxBrake = brake * p.maxBrakeF;
    double Fx = FxDrive - FxBrake - drag;

    // Longitudinal accel estimate for load transfer (uses previous-step
    // vx implicitly via Fx/mass — good enough at 1ms resolution).
    double axEstimate = Fx / p.mass;

    auto [Fzf, Fzr] = axleLoads(s, p, axEstimate);

    // --- Combined slip: friction circle ---
    // Each axle has ONE grip budget shared between longitudinal and
    // lateral force, not two separate budgets. Split the longitudinal
    // demand across axles (front carries braking mostly via brake bias
    // in a real car; here we approximate 50/50 for brake, rear-only
    // for drive since this is a rough RWD-style split).
    double FxFront = -FxBrake * 0.5;         // braking only, front share
    double FxRear = FxDrive - FxBrake * 0.5; // drive + rear brake share

    double FzfMax = p.muTire * Fzf;
    double FzrMax = p.muTire * Fzr;

    // Remaining lateral capacity after longitudinal demand eats into
    // the circle: sqrt(mu*Fz)^2 - Fx^2), clamped at zero.
    auto lateralCapacity = [](double muFzMax, double fxDemand)
    {
        double remaining = muFzMax * muFzMax - fxDemand * fxDemand;
        return remaining > 0.0 ? std::sqrt(remaining) : 0.0;
    };

    double FyfMax = lateralCapacity(FzfMax, FxFront);
    double FyrMax = lateralCapacity(FzrMax, FxRear);

    double Fyf = std::clamp(FyfRaw, -FyfMax, FyfMax);
    double Fyr = std::clamp(FyrRaw, -FyrMax, FyrMax);

    double ax = (Fx - Fyf * std::sin(steer)) / p.mass + s.vy * s.r;
    double ay = (Fyf * std::cos(steer) + Fyr) / p.mass - s.vx * s.r;
    double rDot = (p.a * Fyf * std::cos(steer) - p.b * Fyr) / p.Iz;

    s.vx += ax * dt;
    s.vy += ay * dt;
    s.r += rDot * dt;
    s.yaw += s.r * dt;

    s.x += (s.vx * std::cos(s.yaw) - s.vy * std::sin(s.yaw)) * dt;
    s.y += (s.vx * std::sin(s.yaw) + s.vy * std::cos(s.yaw)) * dt;

    if (s.vx < 1.0)
        s.vx = 1.0;
}