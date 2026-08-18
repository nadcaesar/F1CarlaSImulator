#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include "track/track.h"
#include "vehicle/vehicle.h"

// Runs a fixed maneuver (accelerate 3s, then hold steerDeg) and logs
// telemetry at 100 Hz to outPath. steerDeg past the tires' grip limit
// will show up as a spin (vy/yaw_rate diverging, vx collapsing).
void runScenario(const std::string &name, double steerDeg, const std::string &outPath)
{
    CarParams p;
    State s;

    const double dt = 0.001;
    const double simTime = 10.0;
    const int steps = static_cast<int>(simTime / dt);

    std::ofstream out(outPath);
    out << "time,x,y,yaw,vx,vy,yaw_rate,lat_accel_g,FL,FR,RL,RR\n";

    for (int i = 0; i < steps; ++i)
    {
        double t = i * dt;

        double throttle = 1.0;
        double brake = 0.0;
        double steer = (t > 3.0) ? (steerDeg * PI / 180.0) : 0.0;

        WheelLoads wheels;
        step(s, p, throttle, brake, steer, dt, &wheels);

        if (i % 10 == 0)
        {
            double latG = (s.vx * s.r) / 9.81;
            out << t << "," << s.x << "," << s.y << "," << s.yaw << ","
                << s.vx << "," << s.vy << "," << s.r << "," << latG << ","
                << wheels.FL << "," << wheels.FR << "," << wheels.RL << "," << wheels.RR << "\n";
        }
    }

    out.close();
    std::cout << name << ": final speed " << s.vx * 3.6 << " km/h, wrote " << outPath << "\n";
}

// Grip-limited cornering speed for a given curvature (0 = straight),
// derived from the same friction-circle model as step(): at the speed
// where required centripetal force v^2/R equals available grip
// muTire*(g + liftCoeff*v^2/mass), solving for v gives the cap below.
// A safety margin backs off from that theoretical limit since a real
// corner also demands some longitudinal force (braking/throttle), which
// the friction circle would otherwise have to share.
double gripSpeedCap(double kappa, const CarParams &p, double topSpeedCap, double safetyMargin)
{
    if (kappa < 1e-6)
        return topSpeedCap;

    const double g = 9.81;
    double radius = 1.0 / kappa;
    double denom = kappa - p.muTire * p.liftCoeff / p.mass;
    if (denom <= 0.0)
        return topSpeedCap; // downforce alone supports any speed at this radius

    double vMax = std::sqrt(p.muTire * g / denom);
    return std::min(topSpeedCap, vMax * safetyMargin);
}

// Drives a closed-loop track using pure-pursuit steering and a
// proportional speed controller whose target speed comes from
// gripSpeedCap() applied to the tightest curvature within a braking
// lookahead distance, so the car slows for corners in advance rather
// than reacting once already in them.
void runCircuitScenario(const std::string &name, Track track, double simTime,
                         const std::string &outPath)
{
    CarParams p;
    State s;

    const double steerTimeHeadway = 0.6;    // s, pure-pursuit lookahead scales with speed...
    const double minSteerLookahead = 10.0;  // ...with this floor at low speed
    const double brakingTimeHeadway = 1.5;  // s, braking lookahead scales with speed too --
    const double minSpeedLookahead = 20.0;  // ...30m flat wasn't enough distance to shed
                                             // speed for a tight corner at a 60 m/s approach
    const double topSpeedCap = 60.0;        // m/s, general cruising target on straights
    const double safetyMargin = 0.8;        // back off from the theoretical grip limit
    const double speedKp = 0.2;
    const double maxPedalRatePerSec = 1.0;  // throttle can't jump faster than this

    std::size_t searchIndex = 0; // State's default vx is a reasonable standing-start speed;
                                  // the controller ramps it toward the real target from there.

    const double dt = 0.001;
    const int steps = static_cast<int>(simTime / dt);

    std::ofstream out(outPath);
    out << "time,x,y,yaw,vx,vy,yaw_rate,lat_accel_g,FL,FR,RL,RR\n";

    std::size_t prevSearchIndex = 0;
    int lapCount = 0;
    double prevThrottle = 0.0;

    for (int i = 0; i < steps; ++i)
    {
        double t = i * dt;

        // A fixed lookahead distance gets twitchy at speed (it's a
        // shorter time horizon the faster the car goes), so scale it
        // with vx instead -- a standard pure-pursuit fix.
        double steerLookahead = std::max(minSteerLookahead, steerTimeHeadway * s.vx);
        auto target = track.lookaheadPoint(s.x, s.y, steerLookahead, searchIndex);
        double steer = purePursuitSteer(s.x, s.y, s.yaw, p.a + p.b, target);

        double speedLookahead = std::max(minSpeedLookahead, brakingTimeHeadway * s.vx);
        double kappaAhead = track.maxCurvatureAhead(searchIndex, speedLookahead);
        double targetSpeed = gripSpeedCap(kappaAhead, p, topSpeedCap, safetyMargin);

        double speedErr = targetSpeed - s.vx;
        double throttleRaw = speedErr > 0.0 ? std::min(1.0, speedErr * speedKp) : 0.0;
        double brakeRaw = speedErr < 0.0 ? std::min(1.0, -speedErr * speedKp) : 0.0;

        // Rate-limit throttle only: a real driver rolls onto the power
        // rather than snapping it to full, especially right at corner
        // exit where the tires are still shedding lateral slip from the
        // turn and can't also take a sudden longitudinal hit. Braking is
        // left unlimited — cars can and do brake abruptly, and limiting
        // it leaves no time to shed speed before a tight corner.
        double maxDelta = maxPedalRatePerSec * dt;
        double throttle = std::clamp(throttleRaw, prevThrottle - maxDelta, prevThrottle + maxDelta);
        double brake = brakeRaw;
        prevThrottle = throttle;

        WheelLoads wheels;
        step(s, p, throttle, brake, steer, dt, &wheels);

        // A large backward jump in the nearest-point index means we
        // crossed the seam back to the start of the waypoint list.
        long backwardJump = static_cast<long>(prevSearchIndex) - static_cast<long>(searchIndex);
        if (backwardJump > static_cast<long>(track.length() / 2.0))
        {
            ++lapCount;
            std::cout << name << ": completed lap " << lapCount << " at t=" << t << "s\n";
        }
        prevSearchIndex = searchIndex;

        if (i % 10 == 0)
        {
            double latG = (s.vx * s.r) / 9.81;
            out << t << "," << s.x << "," << s.y << "," << s.yaw << ","
                << s.vx << "," << s.vy << "," << s.r << "," << latG << ","
                << wheels.FL << "," << wheels.FR << "," << wheels.RL << "," << wheels.RR << "\n";
        }
    }

    out.close();
    std::cout << name << ": final speed " << s.vx * 3.6 << " km/h, " << lapCount
              << " lap(s) completed, wrote " << outPath << "\n";
}

// A 7-corner circuit: 3 left, 4 right, one long fast turn per side
// (>140 km/h), the remaining 5 corners slow (<75 km/h) to medium
// (<130 km/h) via tighter radii. Radii were chosen from gripSpeedCap()
// to land in those bands with margin; sweep angles were chosen so the
// signed turning sums to 360 degrees, which is required for any simple
// closed loop with a 4-vs-3 left/right split to actually close.
//
// Summing to 360 degrees only guarantees the path returns to its
// starting HEADING, not its starting POSITION -- those are independent
// constraints. The straight lengths below were solved (not guessed) so
// the path's endpoint exactly matches its start, weighted to keep T6/T7
// close to their original approach length since they need real
// distance to reach "fast" speed; T4/T5 (both already low-speed
// corners) absorbed most of the adjustment instead.
Track buildGrandCircuit()
{
    return Track::fromSegments({
        {147.1, TurnDirection::Right, 20.0, 170.0}, // T1: slow hairpin
        {89.4, TurnDirection::Left, 45.0, 50.0},    // T2: medium
        {260.0, TurnDirection::Right, 45.0, 140.0}, // T3: medium
        {30.0, TurnDirection::Left, 20.0, 60.0},    // T4: slow
        {30.0, TurnDirection::Right, 45.0, 140.0},  // T5: medium
        {84.0, TurnDirection::Left, 95.0, 40.0},    // T6: fast
        {34.4, TurnDirection::Right, 95.0, 60.0},   // T7: fast
    });
}

int main()
{
    // Demonstrates the friction-circle limit: 15 degrees held at ~230 km/h
    // exceeds available grip, so the tires saturate and the car spins.
    runScenario("Grip-limit demo (15 deg)", 15.0, "results/telemetry/run_grip_limit.csv");

    // A steer angle within what the tires can actually support at speed,
    // showing a normal, controlled corner.
    runScenario("Normal corner (3 deg)", 3.0, "results/telemetry/run_normal.csv");

    // Oval test loop driven with pure-pursuit steering + curvature-adaptive speed.
    runCircuitScenario("Oval circuit (pure pursuit)",
                        Track::stadium(/*straightLength=*/120.0, /*radius=*/40.0,
                                       /*segmentsPerCurve=*/60),
                        30.0, "results/telemetry/run_circuit.csv");

    // 7-corner circuit: 3 left / 4 right, one fast turn per side, the
    // rest slow-to-medium.
    runCircuitScenario("Grand circuit lap (pure pursuit)", buildGrandCircuit(), 41.6,
                        "results/telemetry/run_grand_circuit.csv");

    return 0;
}