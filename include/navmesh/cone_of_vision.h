#pragma once

#include <vector>

#include "point.h"
#include "pointf.h"
#include "polygon.h"

namespace NavMesh
{
  class ConeOfVision
  {
  public:
    std::vector<PointF> GetVision(const Point& center, const int radius,
      const int max_angle = 360, const int start_angle = 0, const int angle_step = 1);
    void AddPolygons(const std::vector<Polygon>& polygons_to_add);

  private:
    std::vector<Polygon> polygons_;
  };
}
