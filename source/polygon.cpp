#include "Polygon.h"

#include <algorithm>

namespace NavMesh {
	const double eps = 1e-11;

	Polygon::Polygon() = default;

	Polygon::Polygon(const Polygon&) = default;


	Polygon::Polygon(Polygon&& other) :
		points_(std::move(other.points_)),
		xs_(std::move(other.xs_)),
		top_lines_(std::move(other.top_lines_)),
		bottom_lines_(std::move(other.bottom_lines_))
	{ }

	Polygon::~Polygon() = default;

	Polygon& Polygon::operator=(Polygon&& other)
	{
		points_ = std::move(other.points_);
		xs_ = std::move(other.xs_);
		top_lines_ = std::move(other.top_lines_);
		bottom_lines_ = std::move(other.bottom_lines_);
		return *this;
	}

	// TODO: Add fast path for when the new point is already provided
	// in the correct order. Between last and the first.
	// Make |points| private.
	void Polygon::AddPoint(const Point& a)
	{
		if (points_.size() <= 2) {
			points_.push_back(a);
			xs_.clear();
			OrderCounterClockwiseAndRemoveCollinearPoints();
			return;
		}

		// Check if this point fits nicely after the last one.
		// Fast path for already ordered points being added.
		Point prev_side = points_.back() - points_[points_.size() - 1];
		Point new_side1 = a - points_.back();
		Point new_side2 = points_[0] - a;
		Point next_side = points_[1] - points_[0];
		if ((prev_side ^ new_side1) > eps && (new_side1 ^ new_side2) > eps && (new_side2 ^ next_side) > eps) {
			points_.push_back(a);
			xs_.clear();
			return;
		}

		// Remove ponits, which will be inside the convex hull.
		// Find tangents from the newly added point and make them
		// two new sides, removing all points inside.

		auto ids = GetTangentIds(a);
		int i1 = ids.first;
		int i2 = ids.second;

		// No tangents. May happen if |a| is inside or on the boundary.
		// Only one tangent - this means that |a| coincides with some point.
		if (i1 < 0 || i2 < 0 || i1 == i2)
			return;

		// Check if we are on the side.
		if (i2 == (i1 + 1) % points_.size()) {
			Point v1 = points_[i1] - a;
			Point v2 = points_[i2] - a;
			if (fabs(v1 ^ v2) < eps && v1 * v2 < eps) return;
		}

		int n = points_.size();

		// Erase points from i1+1 to i2-1 inclusive and wrapping.
		int insert_to = 0;
		if (i2 > i1) {
			// Shift points after i2 to immediately after i1.
			int shift = i2 - i1 - 1;
			for (size_t i = i1 + 1; i < points_.size() - shift; ++i) {
				points_[i] = points_[i + shift];
			}
			points_.resize(points_.size() - shift);
			insert_to = i1;
		}
		else {
			// Remove points before i2 or after i1.
			// Shift all remaining points by i2 positions left.
			int total = i1 - i2 + 1;
			for (int i = 0; i < total; ++i) {
				points_[i] = points_[i + i2];
			}
			points_.resize(total);
			insert_to = total - 1;
		}
		points_.resize(points_.size() + 1);
		// Shift points right by 1 to make a place for |a|.
		for (int i = points_.size() - 1; i > insert_to + 1; --i) {
			points_[i] = points_[i - 1];
		}
		points_[insert_to + 1] = a;

		xs_.clear();
	}

	void Polygon::AddPoint(double x, double y)
	{
		AddPoint(Point(x, y));
	}

	bool Polygon::IsInside(const Point& a) const
	{
		if (xs_.empty()) {
			PrepareForFastInsideQueries();
		}

		// Find, which vertical column the point belongs to.
		// It's such i, that |xs_[i] <= a.x && xs_[i+1] > a.x|.

		// Find first |xs_[i]|, larger than |a.x|.
		auto it = std::upper_bound(xs_.begin(), xs_.end(), a.x);

		// Points lying to the left of the first column, or to
		// the right of the last column can't be inside the polygon.
		if (a.x < xs_.front() + eps || a.x > xs_.back() - eps) {
			return false;
		}

		// Previous entry is smaller or equal than |a.x|.
		int i = it - xs_.begin() - 1;
		const auto& top = top_lines_[i];
		const auto& bottom = bottom_lines_[i];
		// Plug a.x into y=k*x+b of the lines to get at
		// that y are the boundaries for the given x.
		// Check that a.y lies strictly between them.
		return (top.first * a.x + top.second > a.y + eps &&
			bottom.first * a.x + bottom.second < a.y - eps);
	}

	int Polygon::Size() const
	{
		return static_cast<int>(points_.size());
	}

	bool Polygon::Intersects(const Segment& s, const std::pair<int, int>& tangents) const
	{
		// No tangents means that s.b lies inside of the polygon.
		// But it's already guaranteed by PathFinder that no endpoint of |s|
		// is inside, so this check can be skipped:
		//   if (tangents.first < 0 || tangents.second < 0) {
		//	   return false;
		//   }

		const Point v = s.e - s.b;
		Point l;
		Point r;
		Point le;
		Point lr;
		double chord_dir;
		double l_dir;
		double r_dir;

		if (tangents.first == tangents.second) {
			int n = points_.size();
			// One end is a vertex of the polygon.
			// Intersection only if the vector is 
			// strictly between two sides.
			l = points_[(tangents.first - 1 + n) % n] - s.b;
			r = points_[(tangents.second + 1) % n] - s.b;
			return (l ^ v) < -eps && (v ^ r) < -eps;
		}

		l = points_[tangents.first] - s.b;
		r = points_[tangents.second] - s.b;
		le = s.e - points_[tangents.first];
		lr = points_[tangents.second] - points_[tangents.first];
		chord_dir = lr ^ le;
		l_dir = l ^ v;
		r_dir = v ^ r;

		// Endpoint lies beyond the chord.
		// Intersection if |v| between |l| and |r| or even touches them.
		return chord_dir > eps && (l ^ v) < -eps && (v ^ r) < -eps;
	}

	void Polygon::Clear()
	{
		points_.clear();
		xs_.clear();
	}

	const Point& Polygon::operator[](size_t i) const
	{
		return points_[i];
	}

	Polygon Polygon::Inflate(double r) const
	{
		// Fast case for no inflation.
		if (r < eps) return *this;

		Polygon res;
		int n = points_.size();

		if (n == 0) {
			return *this;
		}
		else if (n == 1) {
			res.points_.resize(4);
			res.points_[0] = points_[0] + Point(r, r);
			res.points_[1] = points_[0] + Point(-r, r);
			res.points_[2] = points_[0] + Point(-r, -r);
			res.points_[3] = points_[0] + Point(r, -r);
			return res;
		}
		else if (n == 2) {
			res.points_.resize(4);
			Point hor = (points_[1] - points_[0]);
			hor = hor * (r / hor.Len());
			Point vert = hor.Rotate90clockwise();
			res.points_[0] = points_[0] - hor - vert;
			res.points_[1] = points_[0] - hor + vert;
			res.points_[2] = points_[1] + hor + vert;
			res.points_[3] = points_[1] + hor - vert;
			return res;
		}

		// General case.
		// Calculate new points for any poing of the polygon.
		for (int i = 0; i < n; ++i) {
			Point v1 = points_[i] - points_[(i - 1 + n) % n];
			Point v2 = points_[(i + 1) % n] - points_[i];

			// Skip nodes on straigt sides.
			if (fabs(v1 ^ v2) < eps) continue;

			// Get normals and normalize all vectors.
			v1 = v1 * (1.0 / v1.Len());
			Point n1 = v1.Rotate90clockwise();
			v2 = v2 * (1.0 / v2.Len());
			Point n2 = v2.Rotate90clockwise();

			// cos() for angle between two normals.
			double cosa = (n1 * n2);

			// Move sides outward by r (By n1 and n2 respectively).
			// Add two points on them such that line between them is at least r
			// away from the point i.
			// 4 equal right triangles can be noticed with acute angle a/4.
			// We can find the point by moving for r along the normal
			// and r*tg(a/4) along the side toward the i-th point.

			double tga4 = (sqrt((1 - cosa) / 2)) / (1 + sqrt((1 + cosa) / 2));
			double r2 = r * tga4;
			res.points_.push_back(points_[i] + n1 * r + v1 * r2);
			res.points_.push_back(points_[i] + n2 * r - v2 * r2);
		}
		return res;
	}

	void Polygon::PrepareForFastInsideQueries() const
	{
		xs_.clear();
		top_lines_.clear();
		bottom_lines_.clear();
		for (const auto& point : points_) {
			xs_.push_back(point.x);
		}
		std::sort(xs_.begin(), xs_.end());
		xs_.erase(std::unique(xs_.begin(), xs_.end()), xs_.end());

		top_lines_.resize(xs_.size());
		bottom_lines_.resize(xs_.size());

		// |cur_x| should always point to the element in |xs_|,
		// which is equal to |points[i].x|.
		int cur_x = 0;
		// Naively find cur_x for |points[0].x|.
		for (cur_x = 0; cur_x < xs_.size(); ++cur_x) {
			if (xs_[cur_x] == points_[0].x) break;
		}

		for (size_t i = 0; i < points_.size(); ++i) {
			const Point& next = points_[(i + 1) % points_.size()];
			if (next.x > points_[i].x) {
				// Side goes left to right: It's a bottom side.

				// Find equation for current side: y=kx+b.
				double k = (next.y - points_[i].y) / (next.x - points_[i].x);
				double b = points_[i].y - k * points_[i].x;
				auto line = std::make_pair(k, b);
				// Current line is the buttom for all vertical segments,
				// which start strictly before the right end of this side.
				for (cur_x = cur_x; xs_[cur_x] < next.x; ++cur_x) {
					bottom_lines_[cur_x] = line;
				}
			}
			else if (next.x < points_[i].x) {
				// Side goes right to left: It's a top side.

				// Find equation for current side: y=kx+b.
				double k = (next.y - points_[i].y) / (next.x - points_[i].x);
				double b = points_[i].y - k * points_[i].x;
				auto line = std::make_pair(k, b);
				// Current line is the buttom for all vertical segments,
				// which start strictly before current x, and after or at
				// the next x.
				for (cur_x = cur_x - 1; xs_[cur_x] > next.x; --cur_x) {
					top_lines_[cur_x] = line;
				}
				top_lines_[cur_x] = line;
			}
			else {
				// Vertical line. Skip it.
			}
		}

	}

	void Polygon::OrderCounterClockwiseAndRemoveCollinearPoints()
	{
		if (Size() < 3) return;
		if (((points_[1] - points_[0]) ^ (points_[2] - points_[1])) < 0.0) {
			std::reverse(points_.begin(), points_.end());
		}

		// Remove points on sides or concave direction.
		int n = points_.size();
		std::vector<bool> bad_point(n);
		for (int i = 0; i < n; ++i) {
			Point v1 = points_[i] - points_[(i - 1 + n) % n];
			Point v2 = points_[(i + 1) % n] - points_[i];
			double dir = v1 ^ v2;
			if (dir < -eps) {
				//  Concave point.
				bad_point[i] = true;
			}
			else if (dir < eps && v1 * v2 > eps) {
				// Point on side.
				bad_point[i] = true;
			}
		}
		int j = 0;
		for (int i = 0; i < n; ++i) {
			if (!bad_point[i]) {
				points_[j++] = points_[i];
			}
		}
		points_.resize(j);
	}

	std::pair<int, int> Polygon::GetTangentIdsNaive(const Point& a) const
	{
		std::pair<int, int> res = { -1, -1 };
		const int n = points_.size();
		if (n == 1) {
			return { 0, 0 };
		}
		if (n == 2) {
			Point to_0 = points_[0] - a;
			Point to_1 = points_[1] - a;
			double dir = to_0 ^ to_1;
			if (dir < -eps) {
				return { 0, 1 };
			}
			else if (dir > eps) {
				return { 1, 0 };
			}
			else {
				// dir == 0, collinear points.
				if (to_0 * to_1 > -eps) {
					// |a| lies on the line outside of the segment or on the vertex.
					return to_0.Len2() < to_1.Len2() ? std::make_pair(0, 0) : std::make_pair(1, 1);
				}
				else {
					// |a| lies inside the segment.
					return { 0, 1 };
				}
			}
		}
		// Use naive O(|n|) bruteforce algorihtm.
		double dir_prev;
		// Precompute for the first vertex.
		Point vprev = points_[0] - points_[n - 1];
		Point to_vprev = points_[n - 1] - a;
		dir_prev = to_vprev ^ vprev;
		for (int i = 0; i < n; ++i) {
			if (points_[i] == a) {
				// Retrun closest points as the tangent.
				return { i, i };
			}
			Point to_vi = points_[i] - a;
			Point vi = points_[(i + 1) % n] - points_[i];
			double dir_i = to_vi ^ vi;

			// Positive values are for "dark side" sides.
			// Negative values are for front side. 
			// Zeros can separate them on one or both sides.
			// "+" are always present, but there may be no "-" (if |a| is some vertex).

			// First tangent is the  "0/-" vertex, preceeded by "+" vertex
			if (dir_i < eps && dir_prev > eps) {
				// This is tangent going "forward" in along the polygon.
				res.first = i;
			}

			// Second tangent is the "+" vertex, preceeded by "0/-" vertex.
			if (dir_i > eps && dir_prev < eps) {
				// This is tangent going backward along the polygon.
				res.second = i;
			}
			dir_prev = dir_i;
		}

		if (res.first < 0 || res.second < 0) return res;

		// Special case if |a| is on the continuation of a side.
		// The solutions above would find the farest vertex as a tangent end.
		// For path finding algorithms on the graph, however, they must be the closest.
		int next = (res.first + 1) % n;
		Point v = points_[next] - points_[res.first];
		Point to_v = points_[res.first] - a;
		Point v_next = points_[(next + 1) % n] - points_[next];
		Point to_next = points_[next] - a;

		// Should choose the next vertex, if found is "0" vertex and the next is "-/0".
		if (fabs(to_v ^ v) < eps && (to_next ^ v_next) < eps) {
			res.first = next;
		}

		int prev = (res.second + n - 1) % n;
		Point v_prev = points_[res.second] - points_[prev];
		Point to_prev = points_[prev] - a;

		// Should choose the previous vertex, if that previous point is "0" node,
		// but not the same vertex, which the first tangent is pointing at, unless |a| is equal to it.
		if ((res.first != prev || a == points_[prev]) && fabs(to_prev ^ v_prev) < eps) {
			res.second = prev;
		}

		return res;
	}


	std::pair<int, int> Polygon::GetTangentIdsLogarithmic(const Point& a) const
	{
		std::pair<int, int> res = { -1, -1 };
		const int n = points_.size();

		// Value of the point i is (point[i] - a) ^ (point[i+1] - point[i]).
		// It is positive, if the side rotates counter-clockwise from the segment a-point[i].
		// For back or "dark side" sides it is always positive. It can be 0 for one or two nodes,
		// If |a| lies on the continuation of the side/sides/vertex.
		// It can be negative for the front sides.
		// Goal is to find boundaries between "+" and "0/-" continuous regions.
		// The issue is that points are wrapping around, and we can have "+++--+++" or "---++++---" cases.
		// The trick is to look at the value of the beginning and middle in the region. If they are different,
		// we already can figure out which half to discard.
		// If the values are the same we can figure out if we are still in the same region, or jumped over 
		// a different region by looking at relative rotation between points[m]-a and points[l]-a.


		// 1. Search for the left tangent: a "0/-" node, preceeded by "+" node.
		Point vl = points_[1] - points_[0];
		Point to_l = points_[0] - a;
		Point v_prev = points_[0] - points_[n - 1];
		Point to_prev = points_[n - 1] - a;
		double pointing_l = to_l ^ vl;
		double pointing_prev = to_prev ^ v_prev;
		int l = 0;
		int r = n - 1;
		// Explicit check if the first is already the tangent.
		if (pointing_prev > eps && pointing_l < eps) {
			res.first = 0;
			// Check if we could move it forward:
			// if 0-th node is a "0" node and the next is "-/0" node.
			// If both 0-th and 1-st nodes are "0" nodes, then |a == points[1]|.
			if (pointing_l > -eps) {
				Point v_next = points_[2] - points_[1];
				Point to_next = points_[1] - a;
				double pointing_next = to_next ^ v_next;
				if (pointing_next < -eps) {
					res.first = 1;
				}
				else if (pointing_next < eps) {
					return { 1, 1 };
				}
			}
		}
		else {
			// Loop invariant: switch form "+" to "-/0" guaranteed to happen 
			// from between l and r.
			while (l < r - 1) {
				int m = (r + l + 1) / 2;
				// No need to do (m+1)%n, since r-l > 1, it's guaranteed that 
				// l < m < r. Hence m+1 is still a valid index.
				Point vm = points_[m + 1] - points_[m];
				Point to_m = points_[m] - a;
				double pointing_m = to_m ^ vm;
				double lm_dir = to_l ^ to_m;

				// The switch happened between l and m iff:
				// - l-th  is "+" vertex and m-th is "-/0" vertex.
				//   I.e. |pointing_l > eps && pointint_m < eps|
				// - both are "+" vertices and |m| is strictly to the right of |l|.
				//   I.e. |pointing_l > eps && pointint_m > eps && lm_dir < -eps|
				// - both are "0/-" and |m| is non-strictly to the left of "l"
				//   I.e. |pointing_l < eps && pointint_m < eps && lm_dir > -eps|
				//
				// Regarding the strict/non-strict comparison of |lm_dir|: for the second
				// case it doesn't matter, slince |lm_dir| can't be 0 for both "+" points
				// (the only case for zero |lm_dir| is if |a == points[m]| and |l == m-1|,
				// but that means both |pointing_l == 0| and |pointing_m > 0|).
				// For the third case: it's also impossible to end up with zero |lm_dir|
				// because it breaks the loop invariant:
				// that would mean that the switch from "+" to "-/0" vertex happens from |l-1| to |l|.
				// But it's guaranteed to happen from |l| to |r|.
				// For symmetry with the case below -eps is chosen in both cases.
				//
				// Below code is simplification of the 3 conditions above.
				if (((pointing_m < eps) || (lm_dir < -eps)) &&
					((pointing_l > eps) || (lm_dir > -eps))) {
					// Overshoot.
					// |m| is the desired point or something to the right of it.
					r = m;
				}
				else {
					l = m;
					to_l = to_m;
					pointing_l = pointing_m;
				}
			}
			// In case the point is inside
			// bin search will do something random.
			// Check that there's indeed switch from "+" to "0/-"
			// between |l| and |r|

			int next = (r + 1) % n;
			Point to_r = points_[r] - a;
			Point vr = points_[next] - points_[r];
			double pointing_r = to_r ^ vr;
			if (pointing_r < eps && pointing_l > eps) {
				res.first = r;
				// Check if we could move the point forward.
				Point v_next = points_[(next + 1) % n] - points_[next];
				Point to_next = points_[next] - a;
				double pointing_next = to_next ^ v_next;

				// Should choose the next vertex, if found is "0" vertex and the next is "-/0".
				// If the both r and next are 0, then |a = points[r + 1]|
				if (pointing_r > -eps) {
					if (pointing_next < -eps) {
						res.first = next;
					}
					else if (pointing_next < eps) {
						return { next, next };
					}
				}
			}
			else return { -1, -1 };
		}

		// 2. Search for the right tangent: a "+" node, preceeded by "-/0" node.
		vl = points_[1] - points_[0];
		to_l = points_[0] - a;
		v_prev = points_[0] - points_[n - 1];
		to_prev = points_[n - 1] - a;
		pointing_l = to_l ^ vl;
		pointing_prev = to_prev ^ v_prev;
		l = 0;
		r = n - 1;
		// Explicit check if the first is already the tangent.
		if (pointing_prev < eps && pointing_l > eps) {
			res.second = 0;
			// Check if we could move it forward.
			// Can do it iff the previous node is "0" vertex,
			// but not the same vertex, which the first tangent is pointing at, unless |a| is equal to it.
			// But the equality case is already processed after the first binary search.
			if (res.first != n - 1 && pointing_prev > -eps) {
				res.second = n - 1;
			}
		}
		else {
			// Loop invariant: switch form "-/0" to "+" guaranteed to happen 
			// from between l and r.
			while (l < r - 1) {
				int m = (r + l + 1) / 2;
				// No need to do (m+1)%n, since r-l > 1, it's guaranteed that 
				// l < m < r. Hence m+1 is still a valid index.
				Point vm = points_[m + 1] - points_[m];
				Point to_m = points_[m] - a;
				double pointing_m = to_m ^ vm;
				double lm_dir = to_l ^ to_m;

				// The switch happened between l and m iff:
				// - l-th  is "0/-" vertex and m-th is "+" vertex.
				//   I.e. |pointing_l < eps && pointint_m > eps|
				// - both are "+" vertices and |m| is non-strictly to the right of |l|.
				//   I.e. |pointing_l > eps && pointint_m > eps && lm_dir < eps|
				// - both are "0/-" and |m| is strictly to the left of "l"
				//   I.e. |pointing_l < eps && pointint_m < eps && lm_dir > eps|
				//
				// Regarding the strict/non-strict comparison of |lm_dir|: for the second
				// case it doesn't matter, slince |lm_dir| can't be 0 for both "+" points
				// (the only case for zero |lm_dir| is if |a == points[m]| and |l == m-1|,
				// but that means both |pointing_l == 0| and |pointing_m > 0|).
				// For the third case, if |lm_dir == 0|, we must go right, because it's |m+1|
				// which is the second tangent. That's why the condition there is strict.
				//
				// Below code is the simplification of the 3 conditions above.
				if (((pointing_l < eps) || (lm_dir < eps)) &&
					((pointing_m > eps) || (lm_dir > eps))) {
					// Overshoot.
					// |m| is the desired point or something to the right of it.
					r = m;
				}
				else {
					l = m;
					to_l = to_m;
					pointing_l = pointing_m;
				}
			}
			// In case the point is inside
			// bin search will do something random.
			// Check that there's indeed switch from "0/-" to "+"
			// between |l| and |r|
			Point to_r = points_[r] - a;
			Point vr = points_[(r + 1) % n] - points_[r];
			double pointing_r = to_r ^ vr;
			if (pointing_r > eps && pointing_l < eps) {
				res.second = r;
				// Check if we should move the answer backward.
				// Should choose the previous vertex, if that previous point is "0" node,
				// but not the same vertex, which the first tangent is pointing at, unless |a| is equal to it.
				// But the equality case is already processed after the first binary search.
				if ((res.first != l) && pointing_l > -eps) {
					res.second = l;
				}
			}
		}

		return res;
	}

	std::pair<int, int> Polygon::GetTangentIds(const Point& a) const
	{
		if (points_.size() < kMinPointsForLogTangentsAlgo) {
			return  GetTangentIdsNaive(a);
		}
		else {
			return  GetTangentIdsLogarithmic(a);
		}
	}


	bool Polygon::IsTangent(int i, const Point& a) const
	{
		int n = points_.size();
		Point v1 = points_[i] - points_[(i + n - 1) % n];
		Point v = a - points_[i];
		Point v2 = points_[(i + 1) % n] - points_[i];
		double dir1 = (v1 ^ v);
		double dir2 = (v ^ v2);

		// v lies between v1 and v2 or -v lies between v1 and v2.
		// Strict and unstrict checks allow only closest points
		// to be a tangent if |a| is on the continuation of the side.
		// Also, allow the point to be on one of the sides.
		// But not on both of them, since this would mean that the segment is
		// of a zero length. No sense adding this as an edge.
		return (dir1 > -eps && dir2 > eps) ||
			(dir1 < -eps && dir2 < eps) ||
			((fabs(dir2) < eps && v.Len2() < v2.Len2()) ^
				(fabs(dir1) < eps && v.Len2() < v1.Len2()));
	}

}