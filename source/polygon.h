#pragma once
#include "vector"

#include "point.h"
#include "segment.h"

namespace NavMesh {

	class Polygon
	{
	public:
		Polygon();
		Polygon(const Polygon&);
		Polygon(Polygon&&);
		~Polygon();

		Polygon& operator=(Polygon&&);
		Polygon& operator=(const Polygon&);

		// Adds point to the polygon. 
		// If it lies inside the polygon or on the side, 
		// it will be ignored.
		void AddPoint(const Point& a);
		void AddPoint(int x, int y);

		// Checks if |a| is strictly inside the polygon.
		// Returns false if |a| coincides with some vertex 
		// or lies on the side.
		bool IsInside(const Point& a) const;

		// Number of points in the polygon.
		int Size() const;

		// Checks if |s| intersects any side of this polygon.
		// |tangents| are precomputed tangents from |s.b| to the polygon.
		//
		// Note: neither endpoint of |s| could be inside the polygon.
		// In that case this can return wrong values!
		//
		// Only interior of |s| and polygon is counted as intersections.
		// So, |s| can touch the polygon with an end, touch a side or
		// cross a vertex without intersection.
		// This is to allow paths to pass between touching polygons.
		bool Intersects(const Segment& s, const std::pair<int, int>& tangents) const;

		// Removes all points.
		void Clear();

		// Returns corresponding point.
		// No boundary checks are made.
		const Point& operator[](size_t i) const;

		// Inflates the polygon such that resulting boundary
		// is at least |r| units away from the polygon.
		//
		// Constructs Minkowski sum of the polygon and 2*r x 2*r square centered at 0.
		// Increases number of points by at most 4.
		Polygon Inflate(int r) const;

		// Returns ids of two points which are endpoints for
		// two tangents (in different directions).
		// First id is for the left-most point visible from |a|.
		// Second id is for the right-most point.
		//
		// "Closest" points are returned, so if |a| is lying on the
		// continuation of some side, the closest of two vertices
		// would be returned.
		//
		// Returns {-1, -1} if |a| is inside the polygon.
		// Returns {i, i}, if |a| is the i-th vertex of the polygon.
		// If |a| is on the side, returns two consecutive ids.
		std::pair<int, int> GetTangentIds(const Point& a) const;

		// Checks if the segment from |a| to i-th point is tangent to 
		// this polygon. I.e. i-th point is the leftmost or the rightmost,
		// if seen from |a|.
		bool IsTangent(int i, const Point& a) const;

	private:
		friend class PathFinder;

		static const int kMinPointsForLogTangentsAlgo = 4;

		// Points of the polygon.
		// Always in counter-clockwise order.
		std::vector<Point> points_;

		// Precalculates data-structures for O(log |p|) IsInside
		// algorithm.
		// If called, would require O(|p|) additional memory.
		void PrepareForFastInsideQueries() const;


		void OrderCounterClockwiseAndRemoveCollinearPoints();

		// All points x coordinates sorted. Used for fast IsInside algorithm.
		mutable std::vector<int> xs_;
		// Coefficients for top and buttom a*x+b*y+c == 0 lines for each vertical segment.
		// |b| is always positive.
		// i-th entry corresponds for lines between x[i]..x[i+1];
		mutable std::vector<std::pair<std::pair<int, int>, long long>> top_lines_, bottom_lines_;

		std::pair<int, int> GetTangentIdsNaive(const Point& a) const;
		std::pair<int, int> GetTangentIdsLogarithmic(const Point& a) const;
	};

}