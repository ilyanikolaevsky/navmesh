#pragma once

namespace NavMesh {
	class Point
	{
	public:
		Point() : x(0.0), y(0.0) {}
		Point(double x, double y) : x(x), y(y) {}

		Point& operator=(const Point& other) = default;

		Point operator+(const Point& other) const;
		Point operator-(const Point& other) const;

		// Scalar multiplication.
		double operator*(const Point& other) const;

		// Pointtor multiplication.
		double operator^(const Point& other) const;

		// Scale by k.
		Point operator*(double k)	const;

		bool operator==(const Point& other) const;
		bool operator!=(const Point& other) const;
		bool operator<(const Point& other) const;

		// Length of the vector.
		double Len() const;

		// Squared length.
		double Len2() const;

		// Rotate Pointtor.
		Point Rotate90clockwise() const;

		double x, y;
	};

}