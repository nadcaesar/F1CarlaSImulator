#pragma once
#include <utility>

inline constexpr double PI = 3.14159265358979323846;

struct CarParams
{
    double mass = 798.0;        // kg
    double Iz = 1000.0;         // yaw inertia, kg*m^2
    double a = 1.60;            // CG to front axle, m
    double b = 2.00;            // CG to rear axle, m
    double cgHeight = 0.30;     // m
    double trackWidth = 1.60;   // m
    double Cf = 180000.0;       // front cornering stiffness, N/rad
    double Cr = 220000.0;       // rear cornering stiffness, N/rad
    double maxDriveF = 12000.0; // N
    double maxBrakeF = 45000.0; // N
    double dragCoeff = 1.10;
    double liftCoeff = 3.20; // downforce coeff, N per (m/s)^2
    double muTire = 1.60;    // peak friction coefficient
};

struct State
{
    double x = 0.0, y = 0.0;
    double yaw = 0.0;
    double vx = 30.0;
    double vy = 0.0;
    double r = 0.0;
};

// Static weight + aero downforce, front/rear axle. Load transfer
// (longitudinal, from accel/brake) folded in via ax.
std::pair<double, double> axleLoads(const State &s, const CarParams &p, double ax);

// One integration step. Returns nothing; mutates s in place.
void step(State &s, const CarParams &p, double throttle,
          double brake, double steer, double dt);