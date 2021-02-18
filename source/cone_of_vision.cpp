#include "cone_of_vision.h"
#include "polygon.h"
#include "point.h"
#include "pointf.h"

#define _USE_MATH_DEFINES
#include <math.h>

namespace NavMesh
{
  std::vector<PointF> ConeOfVision::GetVision(const Point& center, const int radius,
    const int max_angle, const int start_angle, const int angle_step)
  {
    std::vector<PointF> vision;

    for (int angle = start_angle; angle < start_angle + max_angle; angle += angle_step)
    {
      PointF vision_point(
        center.x + radius * (float)cos(angle * M_PI / 180),
        center.y + radius * (float)sin(angle * M_PI / 180));

      for (auto& polygon : polygons_)
      {
        for (int i = 0; i < polygon.Size(); i++)
        {
          Point p1(polygon[i]);
          Point p2(polygon[(i + 1) % polygon.Size()]);

          Segment vision_segment(center, (Point)vision_point);

          if (vision_segment.Intersects(p1, p2))
          {
            vision_point = vision_segment.GetIntersection(p1, p2);
          }
        }
      }

      vision.emplace_back(vision_point);
    }

    return vision;
  }

  void ConeOfVision::AddPolygons(const std::vector<Polygon>& polygons_to_add)
  {
    polygons_.clear();
    polygons_.reserve(polygons_to_add.size());
    for (auto const& p : polygons_to_add) {
      polygons_.emplace_back(p);
      // Don't add polygons which are not really an obstacle.
      if (polygons_.back().Size() < 1) polygons_.pop_back();
    }
  }
}
