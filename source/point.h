#pragma once

namespace NavMesh {
	class Point
	{
	public:
		Point() : x(0), y(0) {}
		Point(int x, int y) : x(x), y(y) {}

		Point& operator=(const Point& other) = default;

		Point operator+(const Point& other) const;
		Point operator-(const Point& other) const;

		// Scalar multiplication.
		long long operator*(const Point& other) const;

		// Pointtor multiplication.
		long long operator^(const Point& other) const;

		// Scale by k.
		Point operator*(int k) const;

		bool operator==(const Point& other) const;
		bool operator!=(const Point& other) const;
		bool operator<(const Point& other) const;


		// Length of the vector.
		double Len() const;

		// Squared length.
		long long Len2() const;

		int x, y;
	};

}
