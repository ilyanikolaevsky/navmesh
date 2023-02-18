#pragma once

#include "point.h"

namespace NavMesh {
	class PointF
	{
	public:
		PointF() : x(0), y(0) {}
		PointF(float x, float y) : x(x), y(y) {}
		operator Point() const { return Point((int)x, (int)y); }
		float x, y;
	};

}
