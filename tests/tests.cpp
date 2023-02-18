#include "gtest/gtest.h"

#include "point.h"
#include "pointf.h"
#include "segment.h"
#include "polygon.h"
#include "path_finder.h"
#include "cone_of_vision.h"

#include <vector>
#include <cmath>

using namespace NavMesh;

TEST(Point, Subtracts) {
	EXPECT_EQ(Point(10, 30) - Point(3, 7), Point(10 - 3, 30 - 7));
	EXPECT_EQ(Point(100, 300) - Point(100, 300), Point(0, 0));
}

TEST(Point, Adds) {
	EXPECT_EQ(Point(10, 30) + Point(3, 7), Point(10 + 3, 30 + 7));
}

TEST(Point, ScalarMultiplication) {
	EXPECT_EQ(Point(10, 0) * Point(0, 10), 0);
	EXPECT_EQ(Point(10, 1) * Point(-1, 10), 0);
	EXPECT_EQ(Point(10, 10) * Point(10, 10), 200);
	EXPECT_EQ(Point(1, 2) * Point(3, 4), 11);
	EXPECT_EQ(Point(0, 0) * Point(3, 4), 0);
}

TEST(Point, VectorMultiplication) {
	EXPECT_EQ(Point(10, 0) ^ Point(0, 10), 100);
	EXPECT_EQ(Point(10, 1) ^ Point(100, 10), 0);
	EXPECT_EQ(Point(1, 2) ^ Point(3, 4), -2);
	EXPECT_EQ(Point(0, 0) ^ Point(3, 4), 0);
}

TEST(Point, Scaling) {
	EXPECT_EQ(Point(10, 1) * 2, Point(20, 2));
}

TEST(Point, OperationsConsistent) {
	Point a(1, 6);
	Point b(3, 15);
	Point c(18, -5);
	int k = 5;
	EXPECT_EQ((a - b) * c, a * c - b * c);
	EXPECT_EQ((a - b) * k, a * k - b * k);
	EXPECT_EQ((a - b) ^ c, (a ^ c) - (b ^ c));
	EXPECT_EQ((a * k) ^ b, (a ^ b) * k);
}

TEST(Segment, Intersects) {
	Segment s1(Point(0, 0), Point(10, 0));

	// Intersects |s1| in the middle.
	Segment s2(Point(5, 5), Point(5, -5));
	EXPECT_TRUE(s1.Intersects(s2.b, s2.e));
	EXPECT_TRUE(s2.Intersects(s1.b, s1.e));

	// Touches |s1| in the middle.
	s2 = Segment(Point(5, 0), Point(5, 10));
	EXPECT_TRUE(s1.Intersects(s2.b, s2.e));
	// endpoints of the first segment are not counted!
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Touches |s1| on the endpoint.
	s2 = Segment(Point(10, 0), Point(10, 10));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Lines intersect, segments don't.
	s2 = Segment(Point(5, 10), Point(5, 100));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Lines intersect one endpoint, segment don't.
	s2 = Segment(Point(10, 10), Point(10, 100));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Lines parallel.
	s2 = Segment(Point(0, 10), Point(10, 10));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Lines coincide. Separate segments.
	s2 = Segment(Point(20, 0), Point(30, 0));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Lines coincide. Touching segments.
	s2 = Segment(Point(10, 0), Point(20, 0));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));

	// Lines coincide. Overlapping segments.
	s2 = Segment(Point(5, 0), Point(15, 0));
	EXPECT_FALSE(s1.Intersects(s2.b, s2.e));
	EXPECT_FALSE(s2.Intersects(s1.b, s1.e));
}

TEST(Segment, GetIntersection) {
	Segment s1(Point(0, 10), Point(10, 10));
	EXPECT_EQ(Point(5, 10), (Point)s1.GetIntersection(Point(5, 0), Point(5, 15)));
}

TEST(Polygon, ConstructsConvexHullOnAddedPoints) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(1, 5);
	p.AddPoint(0, 10);
	p.AddPoint(7, 3);
	p.AddPoint(6, 9);
	p.AddPoint(0, 9);
	p.AddPoint(10, 0);
	p.AddPoint(10, 10);

	ASSERT_EQ(p.Size(), 4);
	EXPECT_EQ(p[0], Point(0, 0));
	EXPECT_EQ(p[1], Point(10, 0));
	EXPECT_EQ(p[2], Point(10, 10));
	EXPECT_EQ(p[3], Point(0, 10));
}

TEST(Polygon, IgnoresNewPointOnSideOrVertex) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(0, 10);
	p.AddPoint(10, 10);
	p.AddPoint(10, 0);

	p.AddPoint(10, 0);
	ASSERT_EQ(p.Size(), 4);
	EXPECT_EQ(p[0], Point(10, 10));
	EXPECT_EQ(p[1], Point(0, 10));
	EXPECT_EQ(p[2], Point(0, 0));
	EXPECT_EQ(p[3], Point(10, 0));

	p.AddPoint(5, 0);
	ASSERT_EQ(p.Size(), 4);
	EXPECT_EQ(p[0], Point(10, 10));
	EXPECT_EQ(p[1], Point(0, 10));
	EXPECT_EQ(p[2], Point(0, 0));
	EXPECT_EQ(p[3], Point(10, 0));
}

TEST(Polygon, SegmentStaysASegment) {
	Polygon p;
	p.AddPoint(0, 0);
	ASSERT_EQ(p.Size(), 1);
	EXPECT_EQ(p[0], Point(0, 0));

	p.AddPoint(10, 0);
	ASSERT_EQ(p.Size(), 2);
	EXPECT_EQ(p[0], Point(0, 0));
	EXPECT_EQ(p[1], Point(10, 0));

	// Point on the segment.
	p.AddPoint(5, 0);
	ASSERT_EQ(p.Size(), 2);
	EXPECT_EQ(p[0], Point(0, 0));
	EXPECT_EQ(p[1], Point(10, 0));

	// This should extend the segment.
	p.AddPoint(20, 0);
	ASSERT_EQ(p.Size(), 2);
	EXPECT_EQ(p[0], Point(0, 0));
	EXPECT_EQ(p[1], Point(20, 0));

	// Point on the segment.
	p.AddPoint(15, 0);
	ASSERT_EQ(p.Size(), 2);
	EXPECT_EQ(p[0], Point(0, 0));
	EXPECT_EQ(p[1], Point(20, 0));

	// This should extend the segment.
	p.AddPoint(-20, 0);
	ASSERT_EQ(p.Size(), 2);
	EXPECT_EQ(p[0], Point(20, 0));
	EXPECT_EQ(p[1], Point(-20, 0));

	// Finally, some point not on the line.
	p.AddPoint(0, 20);
	EXPECT_EQ(p.Size(), 3);
}

TEST(Polygon, GetsTangentsPoint) {
	Polygon p;
	p.AddPoint(Point(123, 456));

	Point a;
	// All values computed by hand.
	// On continuation of two sides.

	//
	a = Point(5, 10000);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));
	a = Point(100000, 10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));
	a = p[0];
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));
}


TEST(Polygon, GetsTangentsSegment) {
	// Already provided in counter clockwise order.
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(10, 0));

	Point a;
	// All values computed by hand.

	// In the upper semi-plane. 
	a = Point(5, 10000);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 0));
	a = Point(100000, 10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 0));
	a = Point(-100000, 10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 0));

	// In the lower semi-plane. 
	a = Point(5, -10000);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));
	a = Point(100000, -10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));
	a = Point(-100000, -10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));

	// On the line to the right.
	a = Point(20, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 1));

	// On the line to the left.
	a = Point(-40, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));

	// On the segment.
	a = Point(5, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));

	// On the first point.
	a = Point(0, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));

	// On the second point.
	a = Point(10, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 1));
}

TEST(Polygon, GetsTangentsTriangle) {
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(2, 0));
	p.AddPoint(Point(0, 2));

	Point a;

	// On a vertex.
	a = Point(0, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));
	a = Point(2, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 1));
	a = Point(0, 2);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 2));

	// On a side.
	a = Point(1, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));
	a = Point(1, 1);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 2));
	a = Point(0, 1);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 0));


}

TEST(Polygon, GetsTangentsSmallPolygon) {
	// Pentagon in 30x30 square
	// Already provided in counter clockwise order.
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(30, 0));
	p.AddPoint(Point(30, 20));
	p.AddPoint(Point(20, 30));
	p.AddPoint(Point(0, 30));

	Point a;
	// All values computed by hand.
	// On continuation of two sides.
	a = Point(30, 30);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 3));
	a = Point(0, 50);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 4));

	// On a side.
	a = Point(20, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));
	a = Point(30, 10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 2));
	a = Point(25, 25);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 3));
	a = Point(10, 30);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 4));
	a = Point(0, 20);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(4, 0));

	// On a vertex.
	a = Point(0, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 0));
	a = Point(30, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 1));
	a = Point(30, 20);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 2));
	a = Point(20, 30);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 3));
	a = Point(0, 30);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(4, 4));

	// Close to a side.
	a = Point(20, -1);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));
	a = Point(31, 10);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 2));
	a = Point(26, 26);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 3));
	a = Point(10, 31);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 4));
	a = Point(-1, 20);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(4, 0));


	// Very far.
	a = Point(1000000, 10000000);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 4));

	a = Point(20, -10000000);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));

	// On continuation of one side far away.
	a = Point(1000000, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 3));
	a = Point(0, 1000000);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 4));

	// On continuation of one side close.
	a = Point(40, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 2));
	a = Point(0, 40);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 4));

}


TEST(Polygon, GetsTangentsBigPolygon) {
	// Octogon inscribed in 1000x1000 square.

	Polygon p;
	p.AddPoint(Point(300, 0));
	p.AddPoint(Point(700, 0));
	p.AddPoint(Point(1000, 300));
	p.AddPoint(Point(1000, 700));
	p.AddPoint(Point(700, 1000));
	p.AddPoint(Point(300, 1000));
	p.AddPoint(Point(0, 700));
	p.AddPoint(Point(0, 300));


	Point a;

	// On continuation of two sides.
	a = Point(0, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(7, 0));

	// On a side.
	a = Point(500, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));
	// On a side.
	a = Point(0, 500);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(6, 7));

	// On a vertex.
	a = Point(0, 300);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(7, 7));
	a = Point(700, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 1));
	a = Point(1000, 300);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 2));

	// Close to a side.
	a = Point(100, 100);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(7, 0));
	a = Point(500, -100);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 1));

	// On continuation of one side close.
	a = Point(-100, 0);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(6, 0));

	// Not so close to the polygon
	a = Point(1350, 500);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(1, 4));
	a = Point(500, 1350);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 6));
	a = Point(900, 900);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(3, 4));
	a = Point(1100, 1100);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(2, 5));
	a = Point(-100, -100);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(6, 1));
	a = Point(-100, 1100);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(4, 7));
	a = Point(1100, -100);
	EXPECT_EQ(p.GetTangentIds(a), std::make_pair(0, 3));

}



TEST(Polygon, IsTangentRejectsSelfIntersections) {
	// Pentagon in 3x3 square
	// Already provided in counter clockwise order.
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(3, 0));
	p.AddPoint(Point(3, 2));
	p.AddPoint(Point(2, 3));
	p.AddPoint(Point(0, 3));

	EXPECT_FALSE(p.IsTangent(0, Point(3, 3)));
	EXPECT_FALSE(p.IsTangent(0, Point(10, 1)));

}

TEST(Polygon, IsTangentConsistent) {

	Polygon p;
	p.AddPoint(Point(300, 0));
	p.AddPoint(Point(700, 0));
	p.AddPoint(Point(1000, 300));
	p.AddPoint(Point(1000, 700));
	p.AddPoint(Point(700, 1000));
	p.AddPoint(Point(300, 1000));
	p.AddPoint(Point(0, 700));
	p.AddPoint(Point(0, 300));

	const int n = p.Size();

	for (int x = -100; x <= 1200; x += 100) {
		for (int y = -100; y <= 1200; y += 100) {
			Point a(x, y);
			auto ids = p.GetTangentIds(a);
			if (ids.first < 0 || ids.first == ids.second) continue;

			EXPECT_TRUE(p.IsTangent(ids.first, a));
			EXPECT_TRUE(p.IsTangent(ids.second, a));

			if ((ids.second - 1 + n) % n != ids.first)
				EXPECT_FALSE(p.IsTangent((ids.second - 1 + n) % n, a));
			if ((ids.second + 1) % n != ids.first)
				EXPECT_FALSE(p.IsTangent((ids.second + 1) % n, a));

			if ((ids.first - 1 + n) % n != ids.second)
				EXPECT_FALSE(p.IsTangent((ids.first - 1 + n) % n, a));
			if ((ids.first + 1) % n != ids.second)
				EXPECT_FALSE(p.IsTangent((ids.first + 1) % n, a));
		}
	}
}


double GetDistanceFromPointToSegment(Point o, Point s1, Point s2) {
	// Get equation for the line a*x+b*y+c with a^2+b^2=1.
	double a = s2.y - s1.y;
	double b = s1.x - s2.x;
	double d = sqrt(a * a + b * b);
	a /= d;
	b /= d;
	double c = -s1.x * a - s1.y * b;
	// Get signed distance to the line.
	double distance = fabs(a * o.x + b * o.y + c);
	// Check if projection of o lies inside the segment.
	long long dir1 = (o - s1) * (s2 - s1);
	long long dir2 = (o - s2) * (s1 - s2);

	if (dir1 >= 0 && dir2 >= 0) {
		return distance;
	} else {
		return std::min((s1 - o).Len(), (s2 - o).Len());
	}
}

TEST(Polygon, InflateOnePoint) {
	Polygon p;
	Point o(0, 0);
	p.AddPoint(o);
	int kInflationDistance = 10;
	Polygon res = p.Inflate(kInflationDistance);
	ASSERT_EQ(res.Size(), 4);

	// Corners spaced at right angles.
	for (int i = 0; i < 4; ++i) {
		EXPECT_EQ((res[i] - o) * (res[(i + 1) % 4] - o), 0);
	}

	// Corners rotate counter clockwise.
	for (int i = 0; i < 4; ++i) {
		EXPECT_GT((res[i] - o) ^ (res[(i + 1) % 4] - o), 0);
	}

	// Sides are far enough.
	for (int i = 0; i < 4; ++i) {
		double distance = GetDistanceFromPointToSegment(o, res[i], res[(i + 1) % 4]);
		EXPECT_GE(distance, kInflationDistance - 1e-9);
	}

	EXPECT_TRUE(res.IsInside(o));
}

TEST(Polygon, InflateSegment) {
	Polygon p;
	p.AddPoint(100, 304);
	p.AddPoint(108, 254);
	int kInflationDistance = 10;
	const int kExpectedNumPoints = 6;
	Polygon res = p.Inflate(kInflationDistance);
	ASSERT_EQ(res.Size(), kExpectedNumPoints);


	// Sides rotate counter clockwise.
	for (int i = 0; i < kExpectedNumPoints; ++i) {
		EXPECT_GT((res[i] - res[(i + kExpectedNumPoints - 1) % kExpectedNumPoints]) ^
			(res[(i + 1) % kExpectedNumPoints] - res[i]), 0);
	}

	// Sides are far enough around original points.
	for (int j = 0; j < p.Size(); ++j) {
		EXPECT_TRUE(res.IsInside(p[j]));
		for (int i = 0; i < 4; ++i) {
			double distance = GetDistanceFromPointToSegment(p[j], res[i], res[(i + 1) % kExpectedNumPoints]);
			EXPECT_GE(distance, kInflationDistance - 1e-9);
		}
	}
}

TEST(Polygon, InflateTriangle) {
	Polygon p;
	p.AddPoint(100, 304);
	p.AddPoint(108, 254);
	p.AddPoint(50, 101);
	int kInflationDistance = 10;
	const int kExpectedNumPoints = 7;
	Polygon res = p.Inflate(kInflationDistance);
	ASSERT_EQ(res.Size(), kExpectedNumPoints);

	// Sides rotate counter clockwise.
	for (int i = 0; i < kExpectedNumPoints; ++i) {
		EXPECT_GT((res[i] - res[(i + kExpectedNumPoints - 1) % kExpectedNumPoints]) ^
			(res[(i + 1) % kExpectedNumPoints] - res[i]), 0);
	}


	// Sides are far enough around original points.
	for (int j = 0; j < p.Size(); ++j) {
		EXPECT_TRUE(res.IsInside(p[j]));
		for (int i = 0; i < kExpectedNumPoints; ++i) {
			double distance = GetDistanceFromPointToSegment(p[j], res[i], res[(i + 1) % kExpectedNumPoints]);
			EXPECT_GE(distance, kInflationDistance - 1e-9);
		}
	}
}

TEST(Polygon, IsInsideSmallPolygon) {
	// Pentagon in 3x3 square
	// Already provided in counter clockwise order.
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(30, 0));
	p.AddPoint(Point(30, 20));
	p.AddPoint(Point(20, 30));
	p.AddPoint(Point(0, 30));

	Point a;

	// inside points.
	a = Point(20, 20);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(10, 20);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(20, 10);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(5, 5);
	EXPECT_TRUE(p.IsInside(a));

	// On a side.
	a = Point(0, 10);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(25, 25);
	EXPECT_FALSE(p.IsInside(a));

	// On a vertex.
	a = Point(0, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(30, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(30, 20);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(20, 30);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 30);
	EXPECT_FALSE(p.IsInside(a));

	// Outside of x range.
	a = Point(-1000, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(1000, 1000);
	EXPECT_FALSE(p.IsInside(a));

	// Outside above.
	a = Point(20, 100);
	EXPECT_FALSE(p.IsInside(a));

	// Outside below.
	a = Point(10, -100);
	EXPECT_FALSE(p.IsInside(a));

}

TEST(Polygon, IsInsideWorksAfterClear) {
	Polygon p;
	p.AddPoint(Point(-1000, -1000));
	p.AddPoint(Point(1000, -1000));
	p.AddPoint(Point(1000, 1000));
	p.AddPoint(Point(-1000, 1000));
	p.Clear();

	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(10, 0));
	p.AddPoint(Point(5, 10));

	Point a;

	// Inside.
	a = Point(5, 5);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(1, 1);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(9, 1);
	EXPECT_TRUE(p.IsInside(a));

	// On a vertex.
	a = Point(0, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 10);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(5, 10);
	EXPECT_FALSE(p.IsInside(a));

	// On a side.
	a = Point(0, 5);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(7, 6);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(2, 5);
	EXPECT_FALSE(p.IsInside(a));

	// Outside.
	a = Point(5, -1);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 10);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 2);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(1, 4);
	EXPECT_FALSE(p.IsInside(a));
}

TEST(Polygon, IsInsideWorksAfterAddingPoint) {
	Polygon p;

	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(10, 0));
	p.AddPoint(Point(5, 10));

	Point a;

	a = Point(5, 5);
	EXPECT_TRUE(p.IsInside(a));

	p.AddPoint(Point(10, 10));
	p.AddPoint(Point(0, 10));

	// Inside.
	a = Point(5, 5);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(1, 1);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(2, 9);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(9, 9);
	EXPECT_TRUE(p.IsInside(a));

	// Outside.
	a = Point(7, 11);
	EXPECT_FALSE(p.IsInside(a));
}

TEST(Polygon, IsInsideTriangle) {
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(10, 0));
	p.AddPoint(Point(5, 10));

	Point a;

	// Inside.
	a = Point(5, 5);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(1, 1);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(9, 1);
	EXPECT_TRUE(p.IsInside(a));

	// On a vertex.
	a = Point(0, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 10);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(5, 10);
	EXPECT_FALSE(p.IsInside(a));

	// On a side.
	a = Point(0, 5);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(7, 6);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(2, 4);
	EXPECT_FALSE(p.IsInside(a));

	// Outside.
	a = Point(5, -1);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 10);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 2);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(1, 4);
	EXPECT_FALSE(p.IsInside(a));
}

TEST(Polygon, IsInsideSegment) {
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(10, 0));

	Point a;

	// Always outside.
	a = Point(5, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(5, 1000);
	EXPECT_FALSE(p.IsInside(a));
}

TEST(Polygon, IsInsideOnePoint) {
	Polygon p;
	p.AddPoint(Point(0, 0));

	Point a;

	// Always outside.
	a = Point(5, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(5, 1000);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(0, 10);
	EXPECT_FALSE(p.IsInside(a));
}



TEST(Polygon, IsInsideBigPolygon) {
	Polygon p;

	p.AddPoint(Point(0, 10));
	p.AddPoint(Point(10, 0));
	p.AddPoint(Point(30, 20));
	p.AddPoint(Point(20, 30));

	Point a;

	// inside points.
	a = Point(10, 10);
	EXPECT_TRUE(p.IsInside(a));
	a = Point(20, 20);
	EXPECT_TRUE(p.IsInside(a));

	// Above.
	a = Point(5, 30);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 30);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(15, 30);
	EXPECT_FALSE(p.IsInside(a));

	// Below.
	a = Point(5, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(10, 0);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(15, 0);
	EXPECT_FALSE(p.IsInside(a));

	// Corner point.
	a = Point(0, 10);
	EXPECT_FALSE(p.IsInside(a));
	a = Point(30, 20);
	EXPECT_FALSE(p.IsInside(a));
}


TEST(polygon, Intersects) {
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(30, 0));
	p.AddPoint(Point(30, 20));
	p.AddPoint(Point(20, 30));
	p.AddPoint(Point(0, 30));

	Point a;
	Point b;

	// All values checked by hand.

	// Strictly intersects the polygon.
	a = Point(-10, -10);
	b = Point(100, 100);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Strictly intersects the polygon.
	a = Point(20, 0);
	b = Point(20, 30);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Strictly intersects the polygon.
	a = Point(0, -100);
	b = Point(20, 1000);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Strictly intersects the polygon.
	a = Point(30, 30);
	b = Point(-10, 20);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// A chord. Strictly intersects the polygon.
	a = Point(0, 30);
	b = Point(30, 0);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// From a vertex outward.
	a = Point(0, 0);
	b = Point(-20, -20);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// From a vertex inward.
	a = Point(0, 0);
	b = Point(1000, 1000);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Far away segment.
	a = Point(1000, 1000);
	b = Point(1010, 1010);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Parallel to a side.
	a = Point(50, 10);
	b = Point(10, 50);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Parallel to a side.
	a = Point(0, -10);
	b = Point(30, -10);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Passes through the side.
	a = Point(-10, 0);
	b = Point(40, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Part of a side from vertex
	a = Point(30, 0);
	b = Point(10, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// On continuation of a side, but far away.
	a = Point(40, 0);
	b = Point(50, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// On continuation of a side, but far away.
	a = Point(0, 40);
	b = Point(0, 50);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// On a side, in the middle of it.
	a = Point(20, 0);
	b = Point(10, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Touches a side with one end.
	a = Point(20, 0);
	b = Point(20, -100);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Touches a side with one end.
	a = Point(25, 25);
	b = Point(30, 30);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	// Touches the whole side.
	a = Point(40, 10);
	b = Point(10, 40);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
}

TEST(polygon, IntersectsPoint) {
	Polygon p;
	p.AddPoint(Point(0, 0));


	Point a;
	Point b;

	// Allways no intersections no matter that.
	a = Point(-1, -1);
	b = Point(10, 10);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	a = Point(0, 0);
	b = Point(10, 10);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
	a = Point(-1, 0);
	b = Point(1, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
}

TEST(polygon, IntersectsSegment) {
	Polygon p;
	p.AddPoint(Point(0, 0));
	p.AddPoint(Point(10, 0));

	Point a;
	Point b;

	// Parallel segment
	a = Point(0, 10);
	b = Point(10, 10);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));

	// Passes through the end.
	a = Point(0, -5);
	b = Point(0, 5);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));

	// From one end outward.
	a = Point(0, 0);
	b = Point(0, 1);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));

	// Passes through sides.
	a = Point(-1, 0);
	b = Point(11, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));

	// Partly passes through the sides.
	a = Point(-1, 0);
	b = Point(5, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));

	// Intersects in the middle.
	a = Point(5, -5);
	b = Point(5, 5);
	EXPECT_TRUE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_TRUE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));

	// Touches the side.
	a = Point(5, -10);
	b = Point(5, 0);
	EXPECT_FALSE(p.Intersects(Segment(a, b), p.GetTangentIds(a)));
	EXPECT_FALSE(p.Intersects(Segment(b, a), p.GetTangentIds(b)));
}

#define EXPECT_EDGE(edges, seg) EXPECT_TRUE(std::find(edges.begin(), edges.end(), seg) != edges.end())
#define EXPECT_NO_EDGE(edges, seg) EXPECT_TRUE(std::find(edges.begin(), edges.end(), seg) == edges.end())

TEST(PathFinder, NoObstacles) {
	PathFinder pf;
	pf.AddExternalPoints({ Point(0, 0), Point(1,1) });
	std::vector<Segment> edges = pf.GetEdgesForDebug();
	ASSERT_EQ(edges.size(), 1);
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(1, 1)));
}

TEST(PathFinder, OnePolygon) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(1, 0);
	p.AddPoint(0, 1);
	PathFinder pf;
	pf.AddPolygons({ p }, 0);
	std::vector<Segment> edges = pf.GetEdgesForDebug();
	ASSERT_EQ(edges.size(), 3);
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 1)));
	EXPECT_EDGE(edges, Segment(Point(0, 1), Point(1, 0)));
	EXPECT_EDGE(edges, Segment(Point(1, 0), Point(0, 0)));
}

TEST(PathFinder, OnePolygonTwoPoints) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(10, 0);
	p.AddPoint(0, 10);
	PathFinder pf;
	pf.AddPolygons({ p }, 0);
	// Points obstructed by the polygon.
	pf.AddExternalPoints({ Point{10, 10}, Point(-10,-10) });
	std::vector<Segment> edges = pf.GetEdgesForDebug();

	ASSERT_EQ(edges.size(), 7);
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(0, 10), Point(10, 0)));
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(0, 0)));

	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(10, 0)));

	EXPECT_EDGE(edges, Segment(Point(-10, -10), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(-10, -10), Point(10, 0)));

	// Points not obstructed by the polygon.
	// Also checks that consecutive calls to AddExternalPoints clear the map.
	pf.AddExternalPoints({ Point{10, 10}, Point(20, 20) });
	edges = pf.GetEdgesForDebug();

	ASSERT_EQ(edges.size(), 8);
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(0, 10), Point(10, 0)));
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(0, 0)));

	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(10, 0)));

	EXPECT_EDGE(edges, Segment(Point(20, 20), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(20, 20), Point(10, 0)));

	EXPECT_EDGE(edges, Segment(Point(20, 20), Point(10, 10)));

	// One point inside the polygon.
	// Also checks that consecutive calls to AddExternalPoints clear the map.
	pf.AddExternalPoints({ Point{10, 10}, Point(3, 3) });
	edges = pf.GetEdgesForDebug();

	ASSERT_EQ(edges.size(), 5);
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(0, 10), Point(10, 0)));
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(0, 0)));

	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(10, 0)));

	// Both points inside the polygon.
	// Also checks that consecutive calls to AddExternalPoints clear the map.
	pf.AddExternalPoints({ Point{2, 2}, Point(3, 3) });
	edges = pf.GetEdgesForDebug();

	ASSERT_EQ(edges.size(), 3);
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(0, 10), Point(10, 0)));
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(0, 0)));
}

TEST(PathFinder, AllowsTouchingEdgesButNotIntersecting) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(10, 0);
	p.AddPoint(0, 10);
	PathFinder pf;
	pf.AddPolygons({ p }, 0);
	// Points obstructed by the polygon.
	pf.AddExternalPoints({ Point{10, 10}, Point(10, -20), Point(-20, 10), Point(-10, 0), Point(100, 0), Point(5, 5) });

	std::vector<Segment> edges = pf.GetEdgesForDebug();
	// Touching corner.
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(10, -20)));
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(-20, 10)));
	// On the corner.
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(0, 10)));
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(10, 0)));
	// touching side.
	EXPECT_EDGE(edges, Segment(Point(10, 10), Point(5, 5)));
	// Form point on side.
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(5, 5)));
	EXPECT_EDGE(edges, Segment(Point(0, 10), Point(5, 5)));

	//Sweeping side.
	EXPECT_EDGE(edges, Segment(Point(-10, 0), Point(100, 0)));

	// Intersecting
	EXPECT_NO_EDGE(edges, Segment(Point(-10, 0), Point(10, 10)));
}


TEST(PathFinder, CanPassBetweenTwoTouchingSides) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(1, 0);
	p.AddPoint(0, 1);
	Polygon p2;
	p2.AddPoint(0, 0);
	p2.AddPoint(1, 0);
	p2.AddPoint(0, -1);

	PathFinder pf;
	pf.AddPolygons({ p, p2 }, 0);

	std::vector<Segment> edges = pf.GetEdgesForDebug();
	// Shared side.
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 1)));
}

TEST(PathFinder, CanPassBetweenTwoTouchingCorner) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(10, 0);
	p.AddPoint(0, 10);
	Polygon p2;
	p2.AddPoint(5, 0);
	p2.AddPoint(0, -10);
	p2.AddPoint(10, -10);

	PathFinder pf;
	pf.AddPolygons({ p, p2 }, 0);

	std::vector<Segment> edges = pf.GetEdgesForDebug();
	// Side with a corner on it.
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(0, 10)));
	// Can switch from one polygon to another.
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(5, 0)));
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(5, 0)));
}

TEST(PathFinder, CantPassThroughIntersection) {
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(10, 0);
	p.AddPoint(0, 10);
	Polygon p2;
	p2.AddPoint(3, 3);
	p2.AddPoint(0, -10);
	p2.AddPoint(10, -10);

	PathFinder pf;
	pf.AddPolygons({ p, p2 }, 0);

	std::vector<Segment> edges = pf.GetEdgesForDebug();
	// Side with a corner on it.
	EXPECT_NO_EDGE(edges, Segment(Point(0, 0), Point(10, 0)));
	// Can switch from one polygon to another.
	EXPECT_NO_EDGE(edges, Segment(Point(0, 0), Point(3, 3)));
	EXPECT_NO_EDGE(edges, Segment(Point(0, 10), Point(3, 3)));
	EXPECT_NO_EDGE(edges, Segment(Point(10, 0), Point(3, 3)));
}


TEST(PathFinder, BuildsAllTangents) {
	// Two triangles froming a 6-corner star.
	Polygon p;
	p.AddPoint(0, 0);
	p.AddPoint(10, 0);
	p.AddPoint(5, 10);
	Polygon p2;
	p2.AddPoint(5, -2);
	p2.AddPoint(7, 8);
	p2.AddPoint(3, 8);

	PathFinder pf;
	pf.AddPolygons({ p, p2 }, 0);

	std::vector<Segment> edges = pf.GetEdgesForDebug();
	// All 6 tangents.
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(5, -2)));
	EXPECT_EDGE(edges, Segment(Point(0, 0), Point(3, 8)));

	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(5, -2)));
	EXPECT_EDGE(edges, Segment(Point(10, 0), Point(7, 8)));

	EXPECT_EDGE(edges, Segment(Point(5, 10), Point(7, 8)));
	EXPECT_EDGE(edges, Segment(Point(5, 10), Point(3, 8)));

	// All sides are banned due to intersections.
	EXPECT_NO_EDGE(edges, Segment(Point(0, 0), Point(10, 0)));
	EXPECT_NO_EDGE(edges, Segment(Point(10, 0), Point(5, 10)));
	EXPECT_NO_EDGE(edges, Segment(Point(5, 10), Point(0, 0)));

	EXPECT_NO_EDGE(edges, Segment(Point(-5, 2), Point(7, 8)));
	EXPECT_NO_EDGE(edges, Segment(Point(7, 8), Point(3, 8)));
	EXPECT_NO_EDGE(edges, Segment(Point(3, 8), Point(-5, 2)));
}

TEST(ConeOfVision, GetVision) {
	ConeOfVision cov;

	std::vector<PointF> v1 = cov.GetVision(Point(0, 0), 100);
	EXPECT_EQ(v1.size(), 360);

	std::vector<PointF> v2 = cov.GetVision(Point(0, 0), 100, 180);
	EXPECT_EQ(v2.size(), 180);

	std::vector<PointF> v3 = cov.GetVision(Point(0, 0), 100, 180, 0);
	EXPECT_EQ(v3.size(), 180);

	std::vector<PointF> v4 = cov.GetVision(Point(0, 0), 100, 180, 0, 2);
	EXPECT_EQ(v4.size(), 90);
}
