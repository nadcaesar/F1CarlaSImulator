#include "track.h"

#include <cmath>
#include <limits>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

Track::Track(std::vector<std::pair<double, double>> waypoints)
    : pts_(std::move(waypoints)), totalLength_(0.0)
{
    for (std::size_t i = 0; i + 1 < pts_.size(); ++i)
    {
        double dx = pts_[i + 1].first - pts_[i].first;
        double dy = pts_[i + 1].second - pts_[i].second;
        totalLength_ += std::sqrt(dx * dx + dy * dy);
    }
    // Closing segment back to the start.
    if (pts_.size() > 1)
    {
        double dx = pts_.front().first - pts_.back().first;
        double dy = pts_.front().second - pts_.back().second;
        totalLength_ += std::sqrt(dx * dx + dy * dy);
    }
}

Track Track::stadium(double straightLength, double radius, int segmentsPerCurve)
{
    std::vector<std::pair<double, double>> pts;
    const double step = 1.0; // ~1m between waypoints along the straights

    // Straight 1: (0,0) -> (S,0)
    for (double d = 0.0; d < straightLength; d += step)
        pts.emplace_back(d, 0.0);

    // Semicircle around (S,R): angle -90deg -> +90deg, closing the right
    // end of the stadium and reversing heading to -x.
    for (int i = 0; i <= segmentsPerCurve; ++i)
    {
        double theta = -kPi / 2.0 + kPi * i / segmentsPerCurve;
        pts.emplace_back(straightLength + radius * std::cos(theta),
                          radius + radius * std::sin(theta));
    }

    // Straight 2: (S,2R) -> (0,2R)
    for (double d = 0.0; d < straightLength; d += step)
        pts.emplace_back(straightLength - d, 2.0 * radius);

    // Semicircle around (0,R): angle +90deg -> +270deg, closing the left
    // end back onto the start point heading +x again.
    for (int i = 0; i <= segmentsPerCurve; ++i)
    {
        double theta = kPi / 2.0 + kPi * i / segmentsPerCurve;
        pts.emplace_back(radius * std::cos(theta), radius + radius * std::sin(theta));
    }

    return Track(pts);
}

std::pair<double, double> Track::lookaheadPoint(double x, double y, double lookaheadDist,
                                                 std::size_t &searchIndex) const
{
    // Local search window: the closest-point search only looks a short
    // way ahead of the last known index, so it can't snap across the
    // loop to a geometrically-near point on a different part of the
    // track (e.g. the two straights of the stadium run close together).
    const std::size_t windowSize = 60;
    const std::size_t n = pts_.size();

    std::size_t best = searchIndex;
    double bestDist = std::numeric_limits<double>::max();
    for (std::size_t k = 0; k < windowSize; ++k)
    {
        std::size_t idx = (searchIndex + k) % n;
        double dx = pts_[idx].first - x;
        double dy = pts_[idx].second - y;
        double d = dx * dx + dy * dy;
        if (d < bestDist)
        {
            bestDist = d;
            best = idx;
        }
    }
    searchIndex = best;

    double accumulated = 0.0;
    std::size_t idx = best;
    while (accumulated < lookaheadDist)
    {
        std::size_t next = (idx + 1) % n;
        double dx = pts_[next].first - pts_[idx].first;
        double dy = pts_[next].second - pts_[idx].second;
        accumulated += std::sqrt(dx * dx + dy * dy);
        idx = next;
    }
    return pts_[idx];
}

double purePursuitSteer(double x, double y, double yaw, double wheelbase,
                         std::pair<double, double> targetPoint)
{
    double dx = targetPoint.first - x;
    double dy = targetPoint.second - y;

    // Target position in the car's local frame (x forward, y left).
    double localX = dx * std::cos(yaw) + dy * std::sin(yaw);
    double localY = -dx * std::sin(yaw) + dy * std::cos(yaw);

    double ld2 = localX * localX + localY * localY;
    if (ld2 < 1e-6)
        return 0.0;

    return std::atan2(2.0 * wheelbase * localY, ld2);
}
