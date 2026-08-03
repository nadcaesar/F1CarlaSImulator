#pragma once

#include <cstddef>
#include <utility>
#include <vector>

enum class TurnDirection
{
    Left,
    Right
};

// One corner (and the straight leading into it) used to build a track
// with Track::fromSegments().
struct TrackSegment
{
    double straightBefore; // m of straight before this corner
    TurnDirection direction;
    double radius;       // m
    double sweepDegrees; // degrees swept through the corner
};

// A closed-loop sequence of (x, y) waypoints the car can drive around,
// with per-waypoint curvature (0 on straights, 1/radius in a corner) so
// a controller can look ahead and slow down before a corner.
class Track
{
public:
    explicit Track(std::vector<std::pair<double, double>> waypoints,
                    std::vector<double> curvature = {});

    // Builds a closed "stadium" loop (two straights joined by two
    // semicircles) starting at (0,0) heading +x, matching State's
    // default position/heading so a car can drive it from t=0.
    static Track stadium(double straightLength, double radius, int segmentsPerCurve);

    // Builds a closed loop from an ordered list of straight+corner
    // segments, starting at (0,0) heading +x. The segments' signed
    // turning must sum to +-360 degrees for the path to close (true of
    // any simple closed track shape, regardless of how many left vs
    // right corners it has).
    static Track fromSegments(const std::vector<TrackSegment> &segments,
                               double pointSpacing = 1.0);

    double length() const { return totalLength_; }

    // Finds the track point closest to (x, y), searching a local window
    // starting at searchIndex (so the match can't jump to the wrong side
    // of the loop), advances searchIndex to it, then returns the point
    // lookaheadDist further along the track (wrapping past the start).
    std::pair<double, double> lookaheadPoint(double x, double y, double lookaheadDist,
                                              std::size_t &searchIndex) const;

    // Max |curvature| (1/radius, 0 on straights) within aheadDist of
    // fromIndex, walking forward along the track. Used to compute a
    // grip-limited target speed before reaching a corner.
    double maxCurvatureAhead(std::size_t fromIndex, double aheadDist) const;

private:
    std::vector<std::pair<double, double>> pts_;
    std::vector<double> curvature_;
    double totalLength_;
};

// Pure-pursuit steering: the front-wheel angle (radians) that arcs a
// bicycle-model car at (x, y, yaw) with the given wheelbase toward
// targetPoint.
double purePursuitSteer(double x, double y, double yaw, double wheelbase,
                         std::pair<double, double> targetPoint);
