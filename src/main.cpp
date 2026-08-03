#include <iostream>
#include <fstream>
#include <string>
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

int main()
{
    // Demonstrates the friction-circle limit: 15 degrees held at ~230 km/h
    // exceeds available grip, so the tires saturate and the car spins.
    runScenario("Grip-limit demo (15 deg)", 15.0, "results/telemetry/run_grip_limit.csv");

    // A steer angle within what the tires can actually support at speed,
    // showing a normal, controlled corner.
    runScenario("Normal corner (3 deg)", 3.0, "results/telemetry/run_normal.csv");

    return 0;
}