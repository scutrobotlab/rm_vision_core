#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "vc/feature/rune_fan_hump.h"
#include "vc/feature/rune_fan_hump_param.h"
#include <numeric>

using namespace std;
using namespace cv;

HumpDetector::HumpDetector(const vector<Point> &contour, int _start_idx, int _end_idx)
    : points(contour.begin() + _start_idx, contour.begin() + _end_idx),
      start_idx(_start_idx),
      end_idx(_end_idx)
{
    direction = getPointsDirectionVector(points);
    avePoint = accumulate(points.begin(), points.end(), Point2f(0, 0), [](Point2f sum, const Point &p)
                          { return sum + static_cast<Point2f>(p); });
    avePoint /= static_cast<float>(points.size());
}

inline double calculateVertexAngle(const Point2f &a, const Point2f &b, const Point2f &c)
{
    Point2f v1 = b - a, v2 = c - a;
    double n1 = norm(v1), n2 = norm(v2);
    if (n1 < 1e-6 || n2 < 1e-6)
        return 180.0;
    double cosTheta = max(-1.0, min(1.0, v1.dot(v2) / (n1 * n2)));
    return acos(cosTheta) * 180.0 / CV_PI;
}

double HumpDetector::CheckCollinearity(const Point2f &p1, const Point2f &p2, const Point2f &p3)
{
    double angle1 = calculateVertexAngle(p1, p2, p3);
    double angle2 = calculateVertexAngle(p2, p1, p3);
    double angle3 = calculateVertexAngle(p3, p1, p2);
    return 180.0 - max({angle1, angle2, angle3});
}

double HumpDetector::CheckAlignment(const Point2f &dir1, const Point2f &dir2, const Point2f &dir3)
{
    double a1 = atan2(dir1.y, dir1.x);
    double a2 = atan2(dir2.y, dir2.x);
    double a3 = atan2(dir3.y, dir3.x);
    double aligned[3] = {0, a2 - a1, a3 - a1};

    for (int i = 0; i < 3; i++)
        while (aligned[i] < -M_PI)
            aligned[i] += 2 * M_PI;
    for (int i = 0; i < 3; i++)
        while (aligned[i] > M_PI)
            aligned[i] -= 2 * M_PI;

    double mean = (aligned[0] + aligned[1] + aligned[2]) / 3.0;
    double var = 0.0;
    for (int i = 0; i < 3; i++)
        var += (aligned[i] - mean) * (aligned[i] - mean);
    return sqrt(var / 3.0) * 180.0 / CV_PI;
}
