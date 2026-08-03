#include <algorithm>
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
    out << "time,x,y,yaw,vx,vy,yaw_rate,lat_accel_g\n";

    for (int i = 0; i < steps; ++i)
    {
        double t = i * dt;

        double throttle = 1.0;
        double brake = 0.0;
        double steer = (t > 3.0) ? (steerDeg * PI / 180.0) : 0.0;

        step(s, p, throttle, brake, steer, dt);

        if (i % 10 == 0)
        {
            double latG = (s.vx * s.r) / 9.81;
            out << t << "," << s.x << "," << s.y << "," << s.yaw << ","
                << s.vx << "," << s.vy << "," << s.r << "," << latG << "\n";
        }
    }

    out.close();
    std::cout << name << ": final speed " << s.vx * 3.6 << " km/h, wrote " << outPath << "\n";
}

// Drives a closed-loop test track using pure-pursuit steering and a
// simple proportional speed controller, logging telemetry the same way
// runScenario does.
void runCircuitScenario(const std::string &name, const std::string &outPath)
{
    CarParams p;
    State s;

    Track track = Track::stadium(/*straightLength=*/120.0, /*radius=*/40.0,
                                  /*segmentsPerCurve=*/60);

    const double lookaheadDist = 15.0; // m, how far ahead the controller aims
    const double targetSpeed = 25.0;   // m/s, kept modest so the 40m-radius
                                        // turns stay within the tires' grip
    const double speedKp = 0.2;

    s.vx = targetSpeed; // start on-pace instead of braking hard at t=0

    const double dt = 0.001;
    const double simTime = 30.0;
    const int steps = static_cast<int>(simTime / dt);

    std::ofstream out(outPath);
    out << "time,x,y,yaw,vx,vy,yaw_rate,lat_accel_g\n";

    std::size_t searchIndex = 0;
    std::size_t prevSearchIndex = 0;
    int lapCount = 0;

    for (int i = 0; i < steps; ++i)
    {
        double t = i * dt;

        auto target = track.lookaheadPoint(s.x, s.y, lookaheadDist, searchIndex);
        double steer = purePursuitSteer(s.x, s.y, s.yaw, p.a + p.b, target);

        double speedErr = targetSpeed - s.vx;
        double throttle = speedErr > 0.0 ? std::min(1.0, speedErr * speedKp) : 0.0;
        double brake = speedErr < 0.0 ? std::min(1.0, -speedErr * speedKp) : 0.0;

        step(s, p, throttle, brake, steer, dt);

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
                << s.vx << "," << s.vy << "," << s.r << "," << latG << "\n";
        }
    }

    out.close();
    std::cout << name << ": final speed " << s.vx * 3.6 << " km/h, " << lapCount
              << " lap(s) completed, wrote " << outPath << "\n";
}

int main()
{
    // Demonstrates the friction-circle limit: 15 degrees held at ~230 km/h
    // exceeds available grip, so the tires saturate and the car spins.
    runScenario("Grip-limit demo (15 deg)", 15.0, "results/telemetry/run_grip_limit.csv");

    // A steer angle within what the tires can actually support at speed,
    // showing a normal, controlled corner.
    runScenario("Normal corner (3 deg)", 3.0, "results/telemetry/run_normal.csv");

    // Closed-loop test track driven with pure-pursuit steering.
    runCircuitScenario("Circuit lap (pure pursuit)", "results/telemetry/run_circuit.csv");

    return 0;
}