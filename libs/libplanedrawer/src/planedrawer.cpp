
#include "planedrawer.h"

#include <vector>
#include <utility>
#include <queue>
#include <iostream>

using namespace std;


std::ostream& operator << (std::ostream& os, const Line2D& l) {
	os << l.first << "->" << l.second;
	return os;
}

void PlaneDrawer::Bresenham(Coor s, Coor t) {
	int dx = abs(t.x - s.x), dy = abs(t.y - s.y);
	short sx = s.x < t.x ? 1 : -1,
		  sy = s.y < t.y ? 1 : -1;
	int e = dx - dy,
		x = s.x, y = s.y;
	do {
		point(x, y);
		if ((x == t.x) && (y == t.y))
			break;
		int e2 = e + e;
		if (e2 > -dy) e -= dy, x += sx;
		if (e2 < dx) e += dx, y += sy;
	} while (1);
}

void PlaneDrawer::circle(Coor o, int r) {
	int x = 0, y = r,
		d = 1 - r;
	// 1.25 is the same as 1
	while (x <= y) {
		Coor dd(x, y);
		point(o + dd); point(o - dd);
		point(o + !dd); point(o - !dd);
		point(o + ~dd); point(o - ~dd);
		point(o + !~dd); point(o - !~dd);
		if (d < 0) d += x + x + 3;
		else d += x + x - y - y + 5, y --;
		x ++;
	}
}

void PlaneDrawer::cross(Coor o, int r) {
	line(o - Coor(r, r), o + Coor(r, r));
	line(o - Coor(r, -r), o + Coor(r, -r));
}

void PlaneDrawer::arrow(Coor o, double dir, int r) {
	circle(o, 2);
	double dx = r * cos(dir),
		   dy = r * sin(dir);
	line(o, o + Coor(dx, dy));
}

