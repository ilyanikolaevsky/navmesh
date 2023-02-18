#include "point.h"
#include <cmath>

namespace NavMesh {

	Point Point::operator+(const Point& other) const 
	{
		return Point(x + other.x, y + other.y);
	}

	Point Point::operator-(const Point& other) const 
	{
		return Point(x - other.x, y - other.y);
	}

	// Scalar multiplication.
	long long Point::operator*(const Point& other) const 
	{
		return (long long)x * other.x + (long long)y * other.y;
	}

	// Pointtor multiplication.
	long long Point::operator^(const Point& other) const 
	{
		return (long long)x * other.y - (long long)y * other.x;
	}

	Point Point::operator*(int k) const
	{
		return Point(x * k, y * k);
	}

	bool Point::operator==(const Point& other) const
	{
		return x == other.x && y == other.y;
	}

	bool Point::operator!=(const Point& other) const
	{
		return x != other.x || y != other.y;
	}

	bool Point::operator<(const Point& other) const
	{
		return x < other.x || (x  == other.x && y < other.y);
	}

	double Point::Len() const
	{
		return sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y);
	}

	long long Point::Len2() const
	{
		return static_cast<long long>(x) * x + static_cast<long long>(y) * y;
	}

}
