#include "Segment.h"
#include "pointf.h"

namespace NavMesh {

	// Ends of |this| segment are not counted as intersections,
	// but points |a1| and |a2| are counted!
	// Usually this segment - is an edge but points are ends of the
	// polygon side. If intersection is through the polygon point
	// they are counted. If it's the segment end point on the polygon
	// it's fine.
	bool Segment::Intersects(const Point& a1, const Point& a2) const
	{
		Point v1 = e - b;
		Point v2 = a1 - b;
		Point v3 = a2 - b;
		// check that v1 is between v2 and v3 => two vector multiplications have different signs
		// => their multiplication is < 0 for intersections to be possible.
		// v1 is strictly between v2 and v3, because a1 and a2 are not counted as an intersection.
		// thus, a collinear vectors mean that it's a1 or a2 lying on the segment.
		// So 0 vector multiplication may still allow intersection.
		if ((v1 ^ v2) * (v1 ^ v3) > 1e-9) return false;

		Point u1 = a2 - a1;
		Point u2 = b - a1;
		Point u3 = e - a1;
		// collinear vectors here mean no intersection. The |a1|-|a2| segment can pass through the corner.
		if ((u1 ^ u2) * (u1 ^ u3) > -1e-9) return false;
		return true;
	}

  PointF Segment::GetIntersection(const Point& a1, const Point& a2) const
	{
		Point p1 = a1 - a2;
		Point p2 = b - e;
		Point tmp = p1 * (int)(b ^ e) - p2 * (int)(a1 ^ a2); 
		long long k = p2 ^ p1; 
		return PointF((float)(tmp.x / k), (float)(tmp.y / k));
	}

	bool Segment::operator==(const Segment& other)
	{
		return (b == other.b && e == other.e) ||
			(b == other.e && e == other.b);
	}
}