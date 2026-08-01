#include <iostream>
#include <fstream>
#include <cmath>

// ---- Car parameters (roughly F1-spec) ----
struct CarParams {
    double mass        = 798.0;    // kg (FIA minimum weight)
    double Iz          = 1000.0;   // yaw inertia, kg*m^2
    double a           = 1.60;     // CG to front axle, m
    double b           = 2.00;     // CG to rear axle, m
    double Cf          = 180000.0; // front cornering stiffness, N/rad
    double Cr          = 220000.0; // rear cornering stiffness, N/rad
    double maxDriveF   = 12000.0;  // max drive force, N
    double maxBrakeF   = 45000.0;  // max brake force, N
    double dragCoeff   = 1.10;     // 0.5 * rho * Cd * A
};

// ---- What we track over time ----
struct State {
    double x = 0.0, y = 0.0;   // position, m
    double yaw = 0.0;          // heading, rad
    double vx = 30.0;          // longitudinal velocity, m/s
    double vy = 0.0;           // lateral velocity, m/s
    double r  = 0.0;           // yaw rate, rad/s
};

// One physics step using a dynamic bicycle model.
// throttle/brake are 0..1, steer is front wheel angle in radians.
void step(State& s, const CarParams& p, double throttle,
          double brake, double steer, double dt) {

    // Slip angles: the difference between where a tire points
    // and the direction it's actually travelling.
    double alphaF = std::atan2(s.vy + p.a * s.r, s.vx) - steer;
    double alphaR = std::atan2(s.vy - p.b * s.r, s.vx);

    // Linear tire model: lateral force opposes slip angle.
    double Fyf = -p.Cf * alphaF;
    double Fyr = -p.Cr * alphaR;

    // Longitudinal forces
    double drag = p.dragCoeff * s.vx * s.vx;
    double Fx   = throttle * p.maxDriveF - brake * p.maxBrakeF - drag;

    // Newton's second law in the car's own reference frame
    double ax = (Fx - Fyf * std::sin(steer)) / p.mass + s.vy * s.r;
    double ay = (Fyf * std::cos(steer) + Fyr) / p.mass - s.vx * s.r;
    double rDot = (p.a * Fyf * std::cos(steer) - p.b * Fyr) / p.Iz;

    // Integrate
    s.vx  += ax * dt;
    s.vy  += ay * dt;
    s.r   += rDot * dt;
    s.yaw += s.r * dt;

    // Convert body-frame velocity to world-frame position
    s.x += (s.vx * std::cos(s.yaw) - s.vy * std::sin(s.yaw)) * dt;
    s.y += (s.vx * std::sin(s.yaw) + s.vy * std::cos(s.yaw)) * dt;

    if (s.vx < 1.0) s.vx = 1.0;  // avoid divide-by-zero in slip angles
}

int main() {
    CarParams p;
    State s;

    const double dt      = 0.001;  // 1 ms timestep
    const double simTime = 10.0;   // seconds
    const int    steps   = static_cast<int>(simTime / dt);

    std::ofstream out("results/telemetry/run.csv");
    out << "time,x,y,yaw,vx,vy,yaw_rate,lat_accel_g\n";

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;

        // Test manoeuvre: accelerate 3s, then hold a steady 3-degree steer
        double throttle = 1.0;
        double brake    = 0.0;
        double steer    = (t > 3.0) ? (3.0 * M_PI / 180.0) : 0.0;

        step(s, p, throttle, brake, steer, dt);

        if (i % 10 == 0) {   // log at 100 Hz
            double latG = (s.vx * s.r) / 9.81;
            out << t << "," << s.x << "," << s.y << "," << s.yaw << ","
                << s.vx << "," << s.vy << "," << s.r << "," << latG << "\n";
        }
    }

    out.close();
    std::cout << "Done. Final speed: " << s.vx * 3.6 << " km/h\n";
    std::cout << "Wrote results/telemetry/run.csv\n";
    return 0;
}