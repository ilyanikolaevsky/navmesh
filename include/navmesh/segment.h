#pragma once

#include "point.h"
#include "pointf.h"

namespace NavMesh {

	class Segment
	{
	public:
		Segment() : b(0, 0), e(0, 0) {}
		Segment(Point b, Point e) : b(b), e(e) {}

		// Check for intersection with segment |a|-|b|.
		// Endpoints of |this| segment are not counted!
		// |a| and |b| are counted as intersection.
		// Parallel segments are never intersecting!
		// (The segment can end at the polygon and
		// |a| and |b| are always vertices of the polygon and
		// |this| is a tested edge).
		bool Intersects(const Point& a, const Point& b) const;

		PointF GetIntersection(const Point& a, const Point& b) const;

		// Order of point |b| and |e| is ignored.
		bool operator==(const Segment& other);

		Point b, e;
	};

}