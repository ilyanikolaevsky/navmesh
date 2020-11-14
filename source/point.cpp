#include "Point.h"
#include "cmath"

namespace NavMesh {

	Point Point::operator+(const Point& other) const {
		return Point(x + other.x, y + other.y);
	}
	Point Point::operator-(const Point& other) const {
		return Point(x - other.x, y - other.y);
	}

	// Scalar multiplication.
	double Point::operator*(const Point& other) const {
		return (double)x * other.x + y * other.y;
	}

	// Pointtor multiplication.
	double Point::operator^(const Point& other) const {
		return (double)x * other.y - y * other.x;
	}

	Point Point::operator*(double k) const
	{
		return Point(x * k, y * k);
	}

	bool Point::operator==(const Point& other) const
	{
		return fabs(x - other.x) < 1e-9 && fabs(y - other.y) < 1e-9;
	}

	bool Point::operator!=(const Point& other) const
	{
		return fabs(x - other.x) > 1e-9 || fabs(y - other.y) > 1e-9;
	}

	bool Point::operator<(const Point& other) const
	{
		return x < other.x || (fabs(x - other.x) < 1e-9 && y < other.y);
	}

	// Length of the Pointtor.
	double Point::Len() const {
		return sqrt(x * x + y * y);
	}

	double Point::Len2() const
	{
		return x * x + y * y;
	}

	Point Point::Rotate90clockwise() const
	{
		return Point(y, -x);
	}

}