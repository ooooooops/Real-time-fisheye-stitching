
#pragma once

#include "geometry.h"
#include <limits>



class Color: public Vector<float> 
{
	public:
		constexpr explicit Color(float r = 0, float g = 0, float b = 0):
		Vector(r, g, b){}

		explicit Color(const Vector<float>& v):
		Vector<float>(v) {}

		explicit Color(const float* p) : Vector<float>(p) {}

		void normalize();

		Color operator * (float p) const
		{ return Color(x * p, y * p, z * p); }

		Color operator * (const Color& c) const
		{ return Color(x * c.x, y * c.y, z * c.z); }

		Color operator + (const Color &v) const
		{ return Color(x + v.x, y + v.y, z + v.z); }

		static const Color WHITE, BLACK, RED, BLUE, NO;
};

