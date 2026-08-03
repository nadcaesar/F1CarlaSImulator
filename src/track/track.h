#pragma once

#include <cstddef>
#include <utility>
#include <vector>

// A closed-loop sequence of (x, y) waypoints the car can drive around.
class Track
{
public:
    explicit Track(std::vector<std::pair<double, double>> waypoints);

    // Builds a closed "stadium" loop (two straights joined by two
    // semicircles) starting at (0,0) heading +x, matching State's
    // default position/heading so a car can drive it from t=0.
    static Track stadium(double straightLength, double radius, int segmentsPerCurve);

    double length() const { return totalLength_; }

    // Finds the track point closest to (x, y), searching a local window
    // starting at searchIndex (so the match can't jump to the wrong side
    // of the loop), advances searchIndex to it, then returns the point
    // lookaheadDist further along the track (wrapping past the start).
    std::pair<double, double> lookaheadPoint(double x, double y, double lookaheadDist,
                                              std::size_t &searchIndex) const;

private:
    std::vector<std::pair<double, double>> pts_;
    double totalLength_;
};

// Pure-pursuit steering: the front-wheel angle (radians) that arcs a
// bicycle-model car at (x, y, yaw) with the given wheelbase toward
// targetPoint.
double purePursuitSteer(double x, double y, double yaw, double wheelbase,
                         std::pair<double, double> targetPoint);
