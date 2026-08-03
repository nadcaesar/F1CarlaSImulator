#include "track.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

Track::Track(std::vector<std::pair<double, double>> waypoints, std::vector<double> curvature)
    : pts_(std::move(waypoints)), totalLength_(0.0)
{
    curvature_ = curvature.empty() ? std::vector<double>(pts_.size(), 0.0) : std::move(curvature);

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
    std::vector<double> curvature;
    const double step = 1.0; // ~1m between waypoints along the straights
    const double kappa = 1.0 / radius;

    // Straight 1: (0,0) -> (S,0)
    for (double d = 0.0; d < straightLength; d += step)
    {
        pts.emplace_back(d, 0.0);
        curvature.push_back(0.0);
    }

    // Semicircle around (S,R): angle -90deg -> +90deg, closing the right
    // end of the stadium and reversing heading to -x.
    for (int i = 0; i <= segmentsPerCurve; ++i)
    {
        double theta = -kPi / 2.0 + kPi * i / segmentsPerCurve;
        pts.emplace_back(straightLength + radius * std::cos(theta),
                          radius + radius * std::sin(theta));
        curvature.push_back(kappa);
    }

    // Straight 2: (S,2R) -> (0,2R)
    for (double d = 0.0; d < straightLength; d += step)
    {
        pts.emplace_back(straightLength - d, 2.0 * radius);
        curvature.push_back(0.0);
    }

    // Semicircle around (0,R): angle +90deg -> +270deg, closing the left
    // end back onto the start point heading +x again.
    for (int i = 0; i <= segmentsPerCurve; ++i)
    {
        double theta = kPi / 2.0 + kPi * i / segmentsPerCurve;
        pts.emplace_back(radius * std::cos(theta), radius + radius * std::sin(theta));
        curvature.push_back(kappa);
    }

    return Track(std::move(pts), std::move(curvature));
}

Track Track::fromSegments(const std::vector<TrackSegment> &segments, double pointSpacing)
{
    std::vector<std::pair<double, double>> pts;
    std::vector<double> curvature;

    double px = 0.0, py = 0.0, heading = 0.0;

    for (const auto &seg : segments)
    {
        // Straight leading into this corner.
        int straightSteps = std::max(1, static_cast<int>(seg.straightBefore / pointSpacing));
        double dx = std::cos(heading) * pointSpacing;
        double dy = std::sin(heading) * pointSpacing;
        for (int i = 0; i < straightSteps; ++i)
        {
            pts.emplace_back(px, py);
            curvature.push_back(0.0);
            px += dx;
            py += dy;
        }

        // Corner: sign is +1 for a left (CCW) turn, -1 for right (CW).
        // The turn center sits radius meters to that side of the current
        // heading; each waypoint is placed on the circle around it as
        // heading sweeps from its starting value toward +-sweepRad.
        double sign = (seg.direction == TurnDirection::Left) ? 1.0 : -1.0;
        double sweepRad = seg.sweepDegrees * kPi / 180.0;
        double cx = px + sign * seg.radius * (-std::sin(heading));
        double cy = py + sign * seg.radius * (std::cos(heading));

        int arcSteps = std::max(1, static_cast<int>(seg.radius * sweepRad / pointSpacing));
        for (int i = 0; i < arcSteps; ++i)
        {
            double theta = heading + sign * sweepRad * static_cast<double>(i) / arcSteps;
            double nx = -std::sin(theta);
            double ny = std::cos(theta);
            pts.emplace_back(cx - sign * seg.radius * nx, cy - sign * seg.radius * ny);
            curvature.push_back(1.0 / seg.radius);
        }

        heading += sign * sweepRad;
        px = cx - sign * seg.radius * (-std::sin(heading));
        py = cy - sign * seg.radius * (std::cos(heading));
    }

    return Track(std::move(pts), std::move(curvature));
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

double Track::maxCurvatureAhead(std::size_t fromIndex, double aheadDist) const
{
    const std::size_t n = pts_.size();
    double maxK = curvature_[fromIndex];

    double accumulated = 0.0;
    std::size_t idx = fromIndex;
    while (accumulated < aheadDist)
    {
        std::size_t next = (idx + 1) % n;
        double dx = pts_[next].first - pts_[idx].first;
        double dy = pts_[next].second - pts_[idx].second;
        accumulated += std::sqrt(dx * dx + dy * dy);
        idx = next;
        if (curvature_[idx] > maxK)
            maxK = curvature_[idx];
    }
    return maxK;
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
